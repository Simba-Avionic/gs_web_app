import rclpy
from rclpy.node import Node
from gs_interfaces.msg import RocketStatus

class RocketFuellingSimNode(Node):
    def __init__(self):
        super().__init__('sim_node1')
        self.publisher_ = self.create_publisher(RocketStatus, 'topic_name', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = RocketStatus()  # Initialize your message
        # Populate your message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = RocketFuellingSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()