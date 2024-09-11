from rclpy.node import Node
from gs_interfaces.msg import ControlPanelSwitches, TankingCmds, RadiolinkTelemetry
from rclpy import spin, init, shutdown
from gs_ros2_utils import get_node_name

class MrGeneralPublisherNode(Node):
    def __init__(self):
        super().__init__('sim_mr_general_publisher')
        
        self.publisher_cp_switches = self.create_publisher(
            ControlPanelSwitches, get_node_name(ControlPanelSwitches.__name__), 10)
        
        self.publisher_tanking_cmds = self.create_publisher(
            TankingCmds, get_node_name(TankingCmds.__name__), 10)
        
        self.publisher_radiolink_telemetry = self.create_publisher(
            RadiolinkTelemetry, get_node_name(RadiolinkTelemetry.__name__), 10)
        
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg1 = ControlPanelSwitches() 
        msg2 = TankingCmds()
        msg3 = RadiolinkTelemetry()

        msg1.header.stamp = self.get_clock().now().to_msg()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg3.header.stamp = self.get_clock().now().to_msg()

        msg1.header.frame_id = "ControlPanelSwitches"
        msg2.header.frame_id = "TankingCmds"
        msg3.header.frame_id = "RadiolinkTelemetry"

        self.publisher_cp_switches.publish(msg1)
        self.get_logger().info(f'CP: {msg1}')

        self.publisher_tanking_cmds.publish(msg2)
        self.get_logger().info(f'TC: {msg2}')

        self.publisher_radiolink_telemetry.publish(msg3)
        self.get_logger().info(f'RT: {msg3}')

def main(args=None):
    init(args=args)
    node = MrGeneralPublisherNode()
    spin(node)
    node.destroy_node()
    shutdown()

if __name__ == '__main__':
    main()