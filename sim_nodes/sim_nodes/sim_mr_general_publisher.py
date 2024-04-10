import rclpy

from rclpy.node import Node
from gs_interfaces.msg import ControlPanelSwitches, TankingCmds, RadioTelemetry
from rclpy import spin, executors

class MrGeneralPublisherNode(Node):
    def __init__(self):
        super().__init__('sim_mr_general_publisher')
        self.publisher_cp_switches = self.create_publisher(ControlPanelSwitches, 'control_panel/switches', 10)
        self.publisher_tanking_cmds = self.create_publisher(TankingCmds, 'tanking/commands', 10)
        self.publisher_radiolink_telemetry = self.create_publisher(RadioTelemetry, 'radiolink/telemetry', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def timer_callback(self):
        msg1 = ControlPanelSwitches() 
        msg2 = TankingCmds()
        msg3 = RadioTelemetry()

        msg1.header.stamp = self.get_clock().now().to_msg()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg3.header.stamp = self.get_clock().now().to_msg()

        # Populate message
        self.publisher_cp_switches.publish(msg1)
        self.get_logger().info(f'CP: {msg1}')

        self.publisher_tanking_cmds.publish(msg2)
        self.get_logger().info(f'TC: {msg2}')

        self.publisher_radiolink_telemetry.publish(msg3)
        self.get_logger().info(f'RT: {msg3}')

def main(args=None):
    rclpy.init(args=args)
    node = MrGeneralPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()