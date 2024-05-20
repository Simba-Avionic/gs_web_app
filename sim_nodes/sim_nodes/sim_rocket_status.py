import rclpy

from rclpy.node import Node
from gs_interfaces.msg import RocketTelemetry, RocketStatus
from rclpy import spin, executors
from gs_ros2_utils import get_node_name

class SimRocketStatusNode(Node):
    def __init__(self):
        super().__init__('sim_rocket_status')
        self.is_alive = True

        # rocket status is based on rocket telemetry values hence subscription
        self.servo_subscriber = self.create_subscription(
            RocketTelemetry, get_node_name(RocketTelemetry.__name__), self.rtelemetry_callback, 10
        )

        self.publisher_ = self.create_publisher(RocketStatus, 'rocket/status', 10)


    def rtelemetry_callback(self,rtelemetry_msg): # called when data received from subscribed topic
        self.is_alive = not (
            rtelemetry_msg.altitude > 100_000 or
            rtelemetry_msg.velocity > 2137 or
            rtelemetry_msg.pressure > 4_242_424 or
            rtelemetry_msg.acceleration > 200 or
            rtelemetry_msg.temperature > 5600 or # the power of the sun in the palm of my hand
            round(rtelemetry_msg.velocity) == 42 # meaning of life, no need to be alive
        )   

    def publish_load_cells(self):
        rocket_status_msg = RocketStatus() # init
        rocket_status_msg.is_alive = self.is_alive
        rocket_status_msg.header.stamp = self.get_clock().now().to_msg()

        # Populate your message
        self.publisher_.publish(rocket_status_msg)
        if self.is_alive:
            self.get_logger().info('We flyin')
        else:
            self.get_logger().info('Rock et made Ka Boom :(')


def main(args=None):
    rclpy.init(args=args)
    node = SimRocketStatusNode()
    while rclpy.ok():
        node.publish_load_cells()
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()