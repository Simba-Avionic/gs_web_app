from pymavlink import mavutil
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

os.environ["MAVLINK_DIALECT"] = "simba"

class MavlinkReceiver(Node):
    def __init__(self):
        super().__init__('mavlink_receiver')
        self.master = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600, dialect="simba")
        self.get_logger().info("Mavlink connection established. Waiting for heartbeat...")
        # self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received. Waiting for custom messages...")

        # Create ROS 2 publishers for each message type
        self._publishers = {
            'SIMBA_MAX_ALTITUDE': self.create_publisher(Header, 'simba_max_altitude', 10),
            'SIMBA_CMD_VALVE_CONTROL': self.create_publisher(Header, 'simba_cmd_valve_control', 10),
            'SIMBA_HEARTBEAT_1': self.create_publisher(Header, 'simba_heartbeat_1', 10),
            'SIMBA_HEARTBEAT_2': self.create_publisher(Header, 'simba_heartbeat_2', 10),
            'SIMBA_CMD_HOLD': self.create_publisher(Header, 'simba_cmd_hold', 10),
            'SIMBA_CMD_ABORT': self.create_publisher(Header, 'simba_cmd_abort', 10),
            'SIMBA_ACTUATOR': self.create_publisher(Header, 'simba_actuator', 10),
            'SIMBA_TANK_TEMPERATURE_1': self.create_publisher(Header, 'simba_tank_temperature_1', 10),
            'SIMBA_TANK_TEMPERATURE_2': self.create_publisher(Header, 'simba_tank_temperature_2', 10),
            'SIMBA_TANK_PRESSURE_1': self.create_publisher(Header, 'simba_tank_pressure_1', 10),
            'SIMBA_TANK_PRESSURE_2': self.create_publisher(Header, 'simba_tank_pressure_2', 10),
            'SIMBA_GPS': self.create_publisher(Header, 'simba_gps', 10)
        }

        self.receive_custom_message()

    def receive_custom_message(self):
        while rclpy.ok():
            msg = self.master.recv_match(blocking=True)
            if not msg:
                continue
            self.get_logger().info(f"Received message: {msg}")
            self.publish_ros_message(msg)

    def publish_ros_message(self, mavlink_msg):
        msg_type = mavlink_msg.get_type()
        if msg_type in self._publishers:
            header_msg = Header()
            header_msg.stamp = self.get_clock().now().to_msg()
            header_msg.frame_id = 'base_link'
            self._publishers[msg_type].publish(header_msg)

def main(args=None):
    rclpy.init(args=args)
    mavlink_receiver = MavlinkReceiver()
    rclpy.spin(mavlink_receiver)
    mavlink_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()