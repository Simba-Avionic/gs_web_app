from pymavlink import mavutil
import os
import sys
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import gs_interfaces.msg as gs_msgs  # Import custom ROS2 messages

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import utils
from utils.paths import SIMBA_XML_PATH

os.environ["MAVLINK_DIALECT"] = "simba"

class MavlinkReceiver(Node):
    def __init__(self):
        super().__init__('mavlink_receiver')

        # Parse simba.xml to dynamically create publishers
        self._publishers = self.create_publishers_from_xml(SIMBA_XML_PATH)

        # Establish MAVLink connection
        self.master = mavutil.mavlink_connection('/dev/ttyS0', baud=57600, dialect="simba")
        self.get_logger().info("Mavlink connection established. Waiting for heartbeat...")
        # self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received. Waiting for custom messages...")

        self.receive_mav_msgs()

    def create_publishers_from_xml(self, xml_path):
        """Parse simba.xml and create ROS2 publishers dynamically."""
        publishers = {}
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for message in root.findall(".//message"):
            msg_name =  utils.convert_message_name(message.get("name"))
            topic_name = f"mavlink/{message.get('name').lower()}"

            # Dynamically resolve the ROS2 message type
            ros_msg_type = getattr(gs_msgs, msg_name, None)
            if ros_msg_type is None:
                self.get_logger().error(f"Message type {msg_name} not found in gs_interfaces.msg")
                continue

            # Create a publisher for the resolved ROS2 message type
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
        msg_type = mavlink_msg.get_type()
        if msg_type in self._publishers:
            # Dynamically create a ROS2 message instance
            ros_msg_class = self._publishers[msg_type].msg_type
            ros_msg = ros_msg_class()

            # Populate the ROS2 message fields
            for field_name in mavlink_msg.get_fieldnames():
                if hasattr(ros_msg, field_name):
                    setattr(ros_msg, field_name, getattr(mavlink_msg, field_name, None))
                else:
                    self.get_logger().warn(f"Field {field_name} not found in ROS2 message {msg_type}")

            # Publish the ROS2 message
            self._publishers[msg_type].publish(ros_msg)
            self.get_logger().info(f"Published ROS2 message on topic: {msg_type}")

def main(args=None):
    rclpy.init(args=args)
    mavlink_receiver = MavlinkReceiver()
    rclpy.spin(mavlink_receiver)
    mavlink_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()