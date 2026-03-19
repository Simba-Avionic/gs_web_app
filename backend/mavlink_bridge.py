import os
import sys
import time
import rclpy
import serial
import serial.tools.list_ports
import threading
import xml.etree.ElementTree as ET
from rclpy.node import Node
from concurrent.futures import ThreadPoolExecutor

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
sys.path.append(PROJECT_ROOT)
sys.path.append(os.path.join(PROJECT_ROOT, "mavlink"))

try:
    from mavlink.src import mavutil
    import mavlink.src.simba as simba_dialect  # Generated dialect
    from drivers.control_panel_reader import ControlPanelReader
    import gs_interfaces.msg as gs_msgs
    import shared.utils as utils
    from shared.paths import SIMBA_XML_PATH
except ImportError as e:
    print(f"Structure Error: {e}")
    sys.exit(1)


class MavlinkBridge(Node):

    def __init__(self, control_panel_port=None, mavlink_port=None, baudrate=57600, num_retries=3):
        super().__init__('mavlink_bridge')

        self._executor = ThreadPoolExecutor(max_workers=5)

        self._mavlink_publishers = self.create_publishers_from_xml(SIMBA_XML_PATH)
        self._control_panel_reader = ControlPanelReader(control_panel_port)

        ### IMPORTANT ###
        mavutil.mavlink = simba_dialect

        if mavlink_port:
            self.get_logger().info(f"Using specified MAVLink port: {mavlink_port}")
            self.master = mavutil.mavlink_connection(mavlink_port, baud=baudrate, dialect="simba")
        else:
            self.master = self.find_mavlink_connection()

        self.master.mav = simba_dialect.MAVLink(self.master)
        self.get_logger().info("Heartbeat received. Waiting for custom messages...")

        self.gs_switches_publisher = self.create_publisher(gs_msgs.TankingCommands, 'tanking/commands', 10)
        self.get_logger().info("Created publisher for tanking commands")
        
        self.abort_publisher = self.create_publisher(gs_msgs.TankingAbort, 'tanking/abort', 10)
        self.get_logger().info("Created publisher for tanking abort")

        self.tare_publisher = self.create_publisher(gs_msgs.LoadCellsTare, 'tanking/load_cells/tare', 10)
        self.get_logger().info("Created publisher for tare commands")

        self._running = True
        self._receiver_thread = None
        self._start_receiver_thread()

        self._panel_lock = threading.Lock()
        
        self.reader_timer = self.create_timer(0.1, self._read_panel_switches)
        self.rocket_timer = self.create_timer(1, self._handle_rocket_switches)
        self.gs_timer = self.create_timer(0.25, self._handle_gs_switches)
        
        self.get_logger().info("Timers initialized for Control Panel")

    def _start_main_loop_thread(self):
        self._control_thread = threading.Thread(
            target=self._main_loop)
        self._control_thread.daemon = True
        self._control_thread.start()
        self.get_logger().info("Started control panel handler thread")

    def _start_receiver_thread(self):
        self._receiver_thread = threading.Thread(
            target=self._receive_mav_msgs_thread)
        self._receiver_thread.daemon = True
        self._receiver_thread.start()
        self.get_logger().info("Started MAVLink receiver thread")

    def _read_panel_switches(self):
        """Dedicated timer callback to safely read control panel switches."""
        try:
            with self._panel_lock:
                self._control_panel_reader.read_switches()
        except Exception as e:
            self.get_logger().error(f"Error reading control panel: {e}")

    def find_mavlink_connection(self, baudrate=57600, dialect="simba", retry_delay=1):
        while True:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                try:
                    print(f"Trying port: {port.device}")
                    conn = mavutil.mavlink_connection(
                        port.device, baud=baudrate, dialect=dialect)
                    # conn.wait_heartbeat(timeout=timeout)
                    print(f"MAVLink heartbeat received on {port.device}")
                    return conn
                except Exception as e:
                    print(f"Failed on {port.device}: {e}")
            print(
                f"No MAVLink device found. Retrying in {retry_delay} seconds...")
            retry_delay *= 2
            time.sleep(retry_delay)

    # TODO: Do not create topics for messages that are not included in config.json 
    # (eg. GS_HEARTBEAT, ACTUATOR_CMD)
    def create_publishers_from_xml(self, xml_path):
        publishers = {}
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for message in root.findall(".//message"):
            msg_name = utils.convert_message_name(message.get("name"))
            self.get_logger().info(f"Processing message: {msg_name}")

            topic_name = f"mavlink/{message.get('name').lower()}"
            ros_msg_type = getattr(gs_msgs, msg_name, None)

            if ros_msg_type is None:
                self.get_logger().error(
                    f"Message type {msg_name} not found in gs_interfaces.msg")
                continue

            publishers[msg_name] = self.create_publisher(
                ros_msg_type, topic_name, 10)
            self.get_logger().info(
                f"Created publisher for topic: {topic_name}")

        return publishers

    def _receive_mav_msgs_thread(self):
        """Thread function to receive MAVLink messages."""
        self.get_logger().info("MAVLink receiver thread running")
        while self._running and rclpy.ok():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue

                if msg.get_type() == 'BAD_DATA':
                    continue

                # Handle RADIO_STATUS
                if msg.get_type() == 'RADIO_STATUS':
                    # self._handle_radio_status(msg)
                    data = msg.to_dict()
                    self.get_logger().info(f"{data}")
                    continue

                self.get_logger().info(f"Received MAVLink message: {msg}")
                self.publish_ros_msg(msg)
            except Exception as e:
                self.get_logger().error(f"Error in receiver thread: {e}")
                time.sleep(0.25)

    def _handle_radio_status(self, msg):
        """Logs radio signal metrics (RSSI and Noise)."""
        # RSSI values are usually expressed as (value / 1.9) - 127 in dBm for SiK radios
        rssi = msg.rssi
        remrssi = msg.remrssi
        noise = msg.noise
        remnoise = msg.remnoise
        
        self.get_logger().info(
            f"\n[RADIO_STATUS]\n"
            f"  Local RSSI: {rssi} | Noise: {noise}\n"
            f"  Remote RSSI: {remrssi} | Noise: {remnoise}\n"
            f"  Tx Errors: {msg.txbuf}% buffer used"
        )

    def publish_ros_msg(self, mavlink_msg):
        """Publish MAVLink message as a ROS2 message."""
        msg_type = utils.convert_message_name(mavlink_msg.get_type())

        if msg_type in self._mavlink_publishers:
            ros_msg_class = self._mavlink_publishers[msg_type].msg_type
            ros_msg = ros_msg_class()

            # TODO: If mavlink message has a timestamp, use it
            ros_msg.header.stamp = self.get_clock().now().to_msg()

            for field_name in mavlink_msg.get_fieldnames():
                if hasattr(ros_msg, field_name):
                    setattr(ros_msg, field_name, getattr(
                        mavlink_msg, field_name, None))
                else:
                    self.get_logger().warn(
                        f"Field {field_name} not found in ROS2 message {msg_type}")

            self._mavlink_publishers[msg_type].publish(ros_msg)
            self.get_logger().info(
                f"Published ROS2 message on topic: {msg_type}")

    def _handle_rocket_switches(self):
        try:
            with self._panel_lock:
                rocket_switches = self._control_panel_reader.get_rocket_actions()

            if not rocket_switches:
                return

            # ARM / DISARM is a unique case in here
            is_armed = bool(rocket_switches.get(("arm_disarm", "rocket"), 0))
            flags = simba_dialect.SIMBA_GS_ARM if is_armed else simba_dialect.SIMBA_GS_DISARM

            TOGGLE_MAP = {
                ("tank_vent", "rocket"): simba_dialect.SIMBA_GS_VENT_VALVE,
                ("dump", "rocket"): simba_dialect.SIMBA_GS_DUMP_VALVE,
                ("enable_cameras", "rocket"): simba_dialect.SIMBA_GS_CAMERAS,
                ("ignition", "rocket"): simba_dialect.SIMBA_GS_LAUNCH,
                ("abort", "abort"): simba_dialect.SIMBA_GS_ABORT,
            }

            for key, bit_val in TOGGLE_MAP.items():
                if bool(rocket_switches.get(key, 0)):
                    flags |= bit_val

            self.master.mav.simba_gs_heartbeat_send(
                int(time.time() * 1000),
                flags
            )

        except Exception as e:
            self.get_logger().error(f"Error handling ROCKET switches: {e}")


    def _handle_gs_switches(self):
        try:
            with self._panel_lock:
                gs_switches = self._control_panel_reader.get_gs_actions()

            if not gs_switches:
                return
        
            tank_msg = gs_msgs.TankingCommands()
            tank_msg.header.stamp = self.get_clock().now().to_msg()
            tank_msg.header.frame_id = "control_panel_gs"
            
            tank_msg.valve_feed_oxidizer = bool(gs_switches.get(("valve_feed_oxidizer", "gs"), 0))
            tank_msg.valve_feed_pressurizer = bool(gs_switches.get(("valve_feed_pressurizer", "gs"), 0))
            tank_msg.valve_vent_oxidizer = bool(gs_switches.get(("valve_vent_oxidizer", "gs"), 0))
            tank_msg.valve_vent_pressurizer = bool(gs_switches.get(("valve_vent_pressurizer", "gs"), 0))
            tank_msg.decoupler_oxidizer = bool(gs_switches.get(("decoupler_oxidizer", "gs"), 0))
            tank_msg.decoupler_pressurizer = bool(gs_switches.get(("decoupler_pressurizer", "gs"), 0))

            self.gs_switches_publisher.publish(tank_msg)

            tare_msg = gs_msgs.LoadCellsTare()
            tare_msg.header.stamp = self.get_clock().now().to_msg()
            tare_msg.header.frame_id = "control_panel_gs"
            tare_msg.tare_rocket = bool(gs_switches.get(("tare_rocket", "gs"), 0))
            tare_msg.tare_oxidizer = bool(gs_switches.get(("tare_oxidizer", "gs"), 0))
            tare_msg.tare_pressurizer = bool(gs_switches.get(("tare_pressurizer", "gs"), 0))

            self.tare_publisher.publish(tare_msg)

            abort_msg = gs_msgs.TankingAbort()
            abort_msg.header.stamp = self.get_clock().now().to_msg()
            abort_msg.header.frame_id = "control_panel_gs"
            abort_msg.abort = bool(gs_switches.get(("abort", "abort"), 0))
            
            self.abort_publisher.publish(abort_msg)

        except Exception as e:
            self.get_logger().error(f"Error handling GS switches: {e}")

                
    def shutdown(self):
        """Clean shutdown of the MAVLink client."""
        self.get_logger().info("Shutting down MAVLink client...")
        self._running = False
        self._executor.shutdown(wait=True)

        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=3.0)

        if hasattr(self, 'master') and self.master:
            self.master.close()

        if hasattr(self, '_control_panel_reader'):
            self._control_panel_reader.close()


if __name__ == '__main__':
    rclpy.init()
    # mavlink_receiver = MavlinkBridge(control_panel_port="/dev/ttyACM0", mavlink_port="/dev/ttyUSB0")
    mavlink_receiver = MavlinkBridge()

    try:
        rclpy.spin(mavlink_receiver)
    except KeyboardInterrupt:
        print("Received keyboard interrupt, shutting down...")
    except rclpy.executors.ExternalShutdownException:
        pass  # Expected on shutdown
    finally:
        mavlink_receiver.shutdown()
        mavlink_receiver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
