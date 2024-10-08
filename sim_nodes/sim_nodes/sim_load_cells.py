import rclpy

from rclpy.node import Node
from gs_interfaces.msg import ValveServos, LoadCells
from gs_ros2_utils import get_node_name


class SimLoadCellsNode(Node):
    def __init__(self):
        super().__init__('sim_load_cells')
        self.load_cell_1 = 0 # assumes that cells start empty
        self.load_cell_2 = 0
        self.is_full_1 = False
        self.is_full_2 = False

        # load cells values change based on valve servos values
        self.servo_subscriber = self.create_subscription(
            ValveServos, get_node_name(ValveServos.__name__), self.servo_callback, 10
        )

        self.publisher_ = self.create_publisher(LoadCells, 'tanking/load_cells', 10)


    def servo_callback(self,servo_msg): # called when data received from subscribed topic
        if servo_msg.servo1_position > 50:
            self.load_cell_1 += int(servo_msg.servo1_position / 10)
        if servo_msg.servo2_position > 50:
            self.load_cell_2 += int(servo_msg.servo2_position / 10)
            

    def publish_load_cells(self):
        load_cells_msg = LoadCells() # init
        load_cells_msg.load_cell_1 = self.load_cell_1
        load_cells_msg.load_cell_2 = self.load_cell_2
        load_cells_msg.header.stamp = self.get_clock().now().to_msg()
        load_cells_msg.header.frame_id = 'LoadCells'

        # Populate your message
        self.publisher_.publish(load_cells_msg)
        self.get_logger().info(f'Publishing: {load_cells_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = SimLoadCellsNode()
    while rclpy.ok():
        node.publish_load_cells()
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()