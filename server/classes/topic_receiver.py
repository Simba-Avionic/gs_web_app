from rclpy.node import Node
import gs_interfaces.msg

class TopicReceiver(Node):
    def __init__(self, msg_type: str, topic_name: str, interval: int, msg_fields):
        super().__init__(msg_type)

        self.msg_fields = msg_fields

        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, msg_type), topic_name, self.msg_callback, interval)
        
        self.latest_msg = None
        self.last_msg_timestamp = None

    def msg_callback(self, msg):
        self.latest_msg = msg

    def get_msg(self):
        return self.latest_msg