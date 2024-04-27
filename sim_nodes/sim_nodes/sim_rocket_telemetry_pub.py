import rclpy
import random

from rclpy.node import Node
from gs_interfaces.msg import RocketTelemetry
from rclpy import spin, executors
from gs_ros2_utils import get_node_name

class SimRocketTelemetryPubNode(Node):
    def __init__(self):
        super().__init__('sim_rocket_telemetry_pub')
        self.publisher_ = self.create_publisher(RocketTelemetry, get_node_name(RocketTelemetry.__name__), 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # callback called during spin

    def timer_callback(self):
        # Generate random valves sensors data
        temperature_celsius = random.uniform(10, 60) # normally these would be fed from sensor
        altitude_m = random.uniform(0,1000) 
        pressure_hpascal = random.uniform(1000,3000) # arbitrary values
        velocity_m_s = random.uniform(0, 300)
        acceleration_m_s_squared = random.uniform(-10,10)

        msg = RocketTelemetry()  # Initialize message
        
        msg.temperature = temperature_celsius
        msg.altitude = altitude_m
        msg.pressure = pressure_hpascal
        msg.velocity = velocity_m_s
        msg.acceleration = acceleration_m_s_squared
        msg.header.stamp = self.get_clock().now().to_msg()

        # Populate your message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SimRocketTelemetryPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()