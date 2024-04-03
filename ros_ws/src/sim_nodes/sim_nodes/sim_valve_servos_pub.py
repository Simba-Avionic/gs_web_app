import rclpy
import random

from rclpy.node import Node
from gs_interfaces.msg import ValveServos


class SimValveServosPubNode(Node):
    def __init__(self):
        super().__init__('simulated_valve_servos_pub')
        self.publisher_ = self.create_publisher(ValveServos, 'tanking/valves/servos', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # callback called during spin

    def timer_callback(self):
        # Generate random valves sensors data
        servo1_pos = random.randint(0, 100) # normally this would be fed from sensor
        servo2_pos = random.randint(0, 100) # normally this would be fed from sensor
        

        msg = ValveServos()  # Initialize message
        msg.servo1_position = servo1_pos
        msg.servo2_position = servo2_pos

        # Populate your message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SimValveServosPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()