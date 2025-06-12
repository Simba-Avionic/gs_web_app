from pymavlink import mavutil
import os
import sys
import time
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import serial.tools.list_ports
import gs_interfaces.msg as gs_msgs  # Import custom ROS2 messages
from src.control_panel_reader import ControlPanelReader, SWITCH_ACTIONS

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shared.utils as utils
from shared.paths import SIMBA_XML_PATH

os.environ["MAVLINK_DIALECT"] = "simba"

class MavlinkClient(Node):
    def __init__(self):
        super().__init__('mavlink_receiver')

        self._publishers = self.create_publishers_from_xml(SIMBA_XML_PATH)
        self._control_panel_reader = ControlPanelReader()

        self.master = self.find_mavlink_connection()
        self.get_logger().info("Heartbeat received. Waiting for custom messages...")

        self.receive_mav_msgs()

    def find_mavlink_connection(baudrate=57600, dialect="simba", retry_delay=1):
        while True:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                try:
                    print(f"Trying port: {port.device}")
                    conn = mavutil.mavlink_connection(port.device, baud=baudrate, dialect=dialect)
                    # conn.wait_heartbeat(timeout=timeout)
                    print(f"✅ MAVLink heartbeat received on {port.device}")
                    return conn
                except Exception as e:
                    print(f"❌ Failed on {port.device}: {e}")
            print(f"🔄 No MAVLink device found. Retrying in {retry_delay} seconds...")
            retry_delay *= 2 
            time.sleep(retry_delay)

    def create_publishers_from_xml(self, xml_path):
        publishers = {}
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for message in root.findall(".//message"):
            msg_name =  utils.convert_message_name(message.get("name"))

            if "_CMD" in msg_name:
                self.get_logger().info(f"Skipping command message: {msg_name}")
                continue

            topic_name = f"mavlink/{message.get('name').lower()}"
            ros_msg_type = getattr(gs_msgs, msg_name, None)

            if ros_msg_type is None:
                self.get_logger().error(f"Message type {msg_name} not found in gs_interfaces.msg")
                continue

            publishers[msg_name] = self.create_publisher(ros_msg_type, topic_name, 10)
            self.get_logger().info(f"Created publisher for topic: {topic_name}")

        return publishers

    def receive_mav_msgs(self):
        """Continuously receive MAVLink messages and publish them as ROS2 messages."""
        while rclpy.ok():
            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue
            self.get_logger().info(f"Received MAVLink message: {msg}")
            self.publish_ros_msg(msg)

    def publish_ros_msg(self, mavlink_msg):
        """Publish MAVLink message as a ROS2 message."""
        msg_type = utils.convert_message_name(mavlink_msg.get_type())

        if msg_type in self._publishers:
            ros_msg_class = self._publishers[msg_type].msg_type
            ros_msg = ros_msg_class()

            for field_name in mavlink_msg.get_fieldnames():
                if hasattr(ros_msg, field_name):
                    setattr(ros_msg, field_name, getattr(mavlink_msg, field_name, None))
                else:
                    self.get_logger().warn(f"Field {field_name} not found in ROS2 message {msg_type}")

            self._publishers[msg_type].publish(ros_msg)
            self.get_logger().info(f"Published ROS2 message on topic: {msg_type}")


if __name__ == '__main__':
    rclpy.init()
    mavlink_receiver = MavlinkClient()
    rclpy.spin(mavlink_receiver)
    mavlink_receiver.destroy_node()
    rclpy.shutdown()