import rclpy
import random

from rclpy.node import Node
from gs_interfaces.msg import ValveSensors
from rclpy import spin, executors
from gs_ros2_utils import get_node_name

class SimValveSensorsPubNode(Node):
    def __init__(self):
        super().__init__('simulated_valve_sensors_pub')
        self.publisher_ = self.create_publisher(ValveSensors, get_node_name(ValveSensors.__name__), 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # callback called during spin

    def timer_callback(self):
        # Generate random valves sensors data
        temperature_celsius = random.uniform(20, 30) # normally these would be fed from sensor
        temperature_rasp_celsius = random.uniform(20, 30) 
        pressure_bar = random.uniform(1,244) # arbitrary values, not really realistic :)

        msg = ValveSensors()  # Initialize message
        msg.temperature = temperature_celsius
        msg.temperature_raspberry = temperature_rasp_celsius
        msg.pressure = pressure_bar
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ValveSensors"

        # Populate your message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SimValveSensorsPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()