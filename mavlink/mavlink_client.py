
import os
import sys
import time
import rclpy
import threading
from rclpy.node import Node
import xml.etree.ElementTree as ET
import serial.tools.list_ports

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.paths import SIMBA_XML_PATH
import shared.utils as utils

import gs_interfaces.msg as gs_msgs     # Import custom ROS2 messages
from src.control_panel_reader import ControlPanelReader

os.environ["MAVLINK_DIALECT"] = "simba"
from pymavlink import mavutil
import simba


class MavlinkClient(Node):
    def __init__(self, control_panel_port=None, mavlink_port=None, baudrate=9600, num_retries=3):
        super().__init__('mavlink_client')

        self._publishers = self.create_publishers_from_xml(SIMBA_XML_PATH)
        self._control_panel_reader = ControlPanelReader(control_panel_port)

        if mavlink_port:
            self.get_logger().info(f"Using specified MAVLink port: {mavlink_port}")
            self.master = mavutil.mavlink_connection(mavlink_port, baud=baudrate, dialect="simba")
        else:
            self.master = self.find_mavlink_connection()

        self.get_logger().info("Heartbeat received. Waiting for custom messages...")

        self.gs_switches_publisher = self.create_publisher(
            gs_msgs.TankingCommands, 'tanking/commands', 10)
        self.get_logger().info("Created publisher for tanking commands")
        
        self.abort_publisher = self.create_publisher(
            gs_msgs.TankingAbort, 'tanking/abort', 10)
        self.get_logger().info("Created publisher for tanking abort")

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
                    print(f"✅ MAVLink heartbeat received on {port.device}")
                    return conn
                except Exception as e:
                    print(f"❌ Failed on {port.device}: {e}")
            print(
                f"🔄 No MAVLink device found. Retrying in {retry_delay} seconds...")
            retry_delay *= 2
            time.sleep(retry_delay)

    def create_publishers_from_xml(self, xml_path):
        publishers = {}
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for message in root.findall(".//message"):
            msg_name = utils.convert_message_name(message.get("name"))
            self.get_logger().info(f"Processing message: {msg_name}")

            if "Cmd" in msg_name or "Ack" in msg_name:
                self.get_logger().info(f"Skipping command message: {msg_name}")
                continue

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

        if msg_type in self._publishers:
            ros_msg_class = self._publishers[msg_type].msg_type
            ros_msg = ros_msg_class()

            for field_name in mavlink_msg.get_fieldnames():
                if hasattr(ros_msg, field_name):
                    setattr(ros_msg, field_name, getattr(
                        mavlink_msg, field_name, None))
                else:
                    self.get_logger().warn(
                        f"Field {field_name} not found in ROS2 message {msg_type}")

            self._publishers[msg_type].publish(ros_msg)
            self.get_logger().info(
                f"Published ROS2 message on topic: {msg_type}")

    def _main_loop(self):
        """
        Background thread that reads switches and sends commands.
        - For "rocket" category: Only react to state changes (0->1)
        - For "gs" category: Continuously send commands while active
        - For "abort" category: Handle abort switch state
        """
        self.get_logger().info("Control panel handler thread running")

        previous_rocket_states = {}

        while self._running and rclpy.ok():
            try:
                switch_states = self._control_panel_reader.read_switches()

                if not switch_states:
                    time.sleep(0.1)
                    continue

                for action_key, state in switch_states.items():
                    action_name, category = action_key

                    if category == "rocket":
                        # If this is a new action or the state has changed
                        if action_key not in previous_rocket_states or previous_rocket_states[action_key] != state:
                            self.get_logger().info(
                                f"Rocket switch {action_name} changed to {state}")
                            self._handle_rocket_switch_action(
                                action_name, state)

                    elif category == "abort" and state == 1:
                        self.get_logger().debug(f"ABORT!!!")
                        self._handle_abort()

                previous_rocket_states = {
                    key: value for key, value in switch_states.items()
                    if key[1] == "rocket"
                }

                self._handle_gs_switches()

                time.sleep(0.25)

            except Exception as e:
                self.get_logger().error(f"Error in control panel thread: {e}")
                time.sleep(0.5)  # Longer delay after error

    def _handle_rocket_switch_action(self, action_name, switch_state):
        """
        Handle rocket switch actions.
        For state changes: send command 3 times and wait time T for ACK
        For tank_vent: send actuator command once
        """

        state_to_send = None

        try:
            if action_name == "tank_vent":
                self._send_tank_vent_command(switch_state)
                self.get_logger().info(f"Sent tank_vent actuator command")
                return
                
            elif action_name == "arm_disarm":
                if switch_state == 1:
                    # Switch is ON - Send ARM command
                    state_to_send = simba.SIMBA_ROCKET_STATE_ARMED
                    cmd_name = "ARM"
                else:
                    # Switch is OFF - Send DISARM command
                    state_to_send = simba.SIMBA_ROCKET_STATE_DISARMED
                    cmd_name = "DISARM"
                    
                self.get_logger().info(f"Sending {cmd_name} command (state={state_to_send})")

            elif action_name == "ignition":
                state_to_send = simba.SIMBA_ROCKET_STATE_IGNITION
                self.get_logger().info(f"Sending state change command: {action_name} -> state {state_to_send}")

            for attempt in range(3):
                try:
                    msg = self.master.mav.simba_cmd_change_state_encode(state_to_send)
                    self.master.mav.send(msg)
                    self.get_logger().info(f"Sent state change command (attempt {attempt+1}/3)")
                except Exception as e:
                    self.get_logger().error(f"Error sending MAVLink command (attempt {attempt+1}): {e}")
            
            ack_received = self._wait_for_ack(state_to_send, timeout=5.0)
            
            if not ack_received:
                error_msg = f"No acknowledgment received for {action_name} command"
                self.get_logger().error(error_msg)
                raise TimeoutError(error_msg)
            
            self.get_logger().info(f"Successfully processed {action_name} command")

        except Exception as e:
            self.get_logger().error(f"Error handling rocket action {action_name}: {e}")

    def _handle_gs_switches(self):
        
        try:
            gs_switches = self._control_panel_reader.get_gs_actions()
        
            msg = gs_msgs.TankingCommands()
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "control_panel_gs"
            
            msg.main_valve = gs_switches.get("main_valve")
            msg.hose_vent = gs_switches.get("hose_vent")
            msg.decoupler = gs_switches.get("decoupler")
            msg.oxidizer_switch = gs_switches.get("N2O_He_switch")

            self.gs_switches_publisher.publish(msg)
            self.get_logger().info(f"Published GS switches: {msg}")

        except Exception as e:
            self.get_logger().error(f"Error handling GS switches: {e}")

    def _handle_abort(self):
        """
        Emergency abort handler. Sends abort commands rapidly through both MAVLink and ROS.
        This is a critical safety function that should operate with highest priority.
        """
        self.get_logger().error("⚠️ ABORT TRIGGERED ⚠️")
        
        # Define abort parameters
        time_period = 10   # Time period in seconds to send abort commands
        interval = 0.1     # Interval between sends in seconds
        
        try:
            # Start time tracking
            start_time = time.time()
            
            # Send abort commands repeatedly for time_period
            while time.time() - start_time < time_period:
                # 1. Send MAVLink abort command
                try:
                    abort_state = simba.SIMBA_ROCKET_STATE_ABORTED
                    msg = self.master.mav.simba_cmd_change_state_encode(abort_state)
                    self.master.mav.send(msg)
                    self.get_logger().info("Sent MAVLink ABORT command")
                except Exception as e:
                    self.get_logger().error(f"Error sending MAVLink ABORT command: {e}")
                
                # 2. Publish ROS abort message
                try:
                    abort_msg = gs_msgs.TankingAbort()
                    abort_msg.header.stamp = self.get_clock().now().to_msg()
                    abort_msg.header.frame_id = "emergency_abort"
                    abort_msg.abort = True
                    
                    self.abort_publisher.publish(abort_msg)
                    self.get_logger().info("Published ROS ABORT message")
                except Exception as e:
                    self.get_logger().error(f"Error publishing ROS ABORT message: {e}")
                
                # Short delay before next send
                time.sleep(interval)
            
            # After rapid-fire period, send one final abort with acknowledgment check
            try:
                # Send final MAVLink abort command
                abort_state = simba.SIMBA_ROCKET_STATE_ABORTED
                msg = self.master.mav.simba_cmd_change_state_encode(abort_state)
                self.master.mav.send(msg)
                
                # Wait for acknowledgment
                ack_received = self._wait_for_ack(abort_state, timeout=2.0)
                if not ack_received:
                    self.get_logger().error("No acknowledgment received for final ABORT command")
                else:
                    self.get_logger().info("Abort sequence acknowledged by rocket")
            except Exception as e:
                self.get_logger().error(f"Error in final ABORT verification: {e}")
            
            self.get_logger().info("Abort sequence completed")
            
        except Exception as e:
            self.get_logger().error(f"Critical error in abort handler: {e}")
            # As a last resort, try to send one more abort command
            try:
                msg = self.master.mav.simba_cmd_change_state_encode(simba.SIMBA_ROCKET_STATE_ABORTED)
                self.master.mav.send(msg)
            except:
                pass


    def _send_tank_vent_command(self, value):
        """Send actuator command."""
        try:
            msg = self.master.mav.simba_actuator_cmd_encode(0x04, value)
            self.master.mav.send(msg)
            self.get_logger().info(f"Sent MAVLink ACTUATOR {1} command with value {value}")
        except Exception as e:
            self.get_logger().error(f"Error sending MAVLink ACTUATOR command: {e}")

    def shutdown(self):
        """Clean shutdown of the MAVLink client."""
        self.get_logger().info("Shutting down MAVLink client...")
        self._running = False

        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2.0)

        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)

        if hasattr(self, 'master') and self.master:
            self.master.close()

        if hasattr(self, '_control_panel_reader'):
            self._control_panel_reader.close()


if __name__ == '__main__':
    rclpy.init()
    mavlink_receiver = MavlinkClient()

    try:
        rclpy.spin(mavlink_receiver)
    except KeyboardInterrupt:
        print("Received keyboard interrupt, shutting down...")
    finally:
        mavlink_receiver.shutdown()
        mavlink_receiver.destroy_node()
        rclpy.shutdown()
