import rclpy
import random

from rclpy.node import Node
from gs_interfaces.msg import Telemetry433
from std_msgs.msg import Header

class SimRadio433PubNode(Node):
    def __init__(self):
        super().__init__('simulated_valve_sensors_pub')
        self.publisher_ = self.create_publisher(Telemetry433, 'radio_433/telemetry', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # callback called during spin

    def timer_callback(self):
        # Generate random radio data to publish
        rssi = random.uniform(-90, 0) # normally these would be fed from sensor
        rssi_remote = random.uniform(-90, 0) 
        noise = random.uniform(-90, 0)

        msg = Telemetry433()  # Initialize message
        msg.header.frame_id = 'dupa'
        msg.rssi = rssi
        msg.rssi_remote = rssi_remote
        msg.noise = noise

        # Populate your message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SimRadio433PubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()