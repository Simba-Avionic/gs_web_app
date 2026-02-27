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


class MavlinkClient(Node):

    def __init__(self, control_panel_port=None, mavlink_port=None, baudrate=57600, num_retries=3):
        super().__init__('mavlink_client')

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

        self.gs_switches_publisher = self.create_publisher(
            gs_msgs.TankingCommands, 'tanking/commands', 10)
        self.get_logger().info("Created publisher for tanking commands")
        
        self.abort_publisher = self.create_publisher(
            gs_msgs.TankingAbort, 'tanking/abort', 10)
        self.get_logger().info("Created publisher for tanking abort")

        self.tare_publisher = self.create_publisher(
            gs_msgs.LoadCellsTare, 'tanking/load_cells/tare', 10)
        self.get_logger().info("Created publisher for tare commands")

        self._running = True
        self._receiver_thread = None
        self._start_receiver_thread()

        self._control_thread = None
        self._start_main_loop_thread()

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

    def create_publishers_from_xml(self, xml_path):
        publishers = {}
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for message in root.findall(".//message"):
            msg_name = utils.convert_message_name(message.get("name"))
            self.get_logger().info(f"Processing message: {msg_name}")

            # if "Cmd" in msg_name:
            #     self.get_logger().info(f"Skipping command message: {msg_name}")
            #     continue

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

                self.get_logger().info(f"Received MAVLink message: {msg}")
                self.publish_ros_msg(msg)
            except Exception as e:
                self.get_logger().error(f"Error in receiver thread: {e}")
                # Prevent tight loop in case of repeated errors
                time.sleep(0.25)

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

    def _main_loop(self):
        """
        Background thread that reads switches and sends commands.
        """
        self.get_logger().info("Control panel handler thread running")

        while self._running and rclpy.ok():
            try:
                switch_states = self._control_panel_reader.read_switches()
                if not switch_states:
                    time.sleep(0.1)
                    continue
                
                self._handle_rocket_switches()
                self._handle_gs_switches()
                time.sleep(0.1)

            except Exception as e:
                self.get_logger().error(f"Error in control panel thread: {e}")
                time.sleep(0.5)  # Longer delay after error

    def _handle_rocket_switches(self):
        try:
            rocket_switches = self._control_panel_reader.get_rocket_actions();

            if rocket_switches is None:
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
            # print(f"MAVLink Flags: {flags:08b}")

        except Exception as e:
            self.get_logger().error(f"Error handling ROCKET switches: {e}")


    def _handle_gs_switches(self):
        
        try:
            gs_switches = self._control_panel_reader.get_gs_actions()

            if gs_switches is None:
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

        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=3.0)

        if hasattr(self, 'master') and self.master:
            self.master.close()

        if hasattr(self, '_control_panel_reader'):
            self._control_panel_reader.close()


if __name__ == '__main__':
    rclpy.init()
    # mavlink_receiver = MavlinkClient(control_panel_port="/dev/ttyACM0", mavlink_port="/dev/ttyUSB0")
    mavlink_receiver = MavlinkClient()

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
