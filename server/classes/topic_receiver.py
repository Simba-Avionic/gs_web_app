from rclpy.node import Node
import std_msgs.msg
# TODO: import message definitions


class TopicReceiver(Node):
    def __init__(self, msg_type: str, topic_name: str, interval: int, msg_fields):
        super().__init__(msg_type)

        self.msg_fields = msg_fields

        self.subscription = self.create_subscription(
            getattr(std_msgs.msg, msg_type), topic_name, self.msg_callback, interval)
        
        self.latest_msg = None
        self.last_msg_timestamp = None

    def msg_callback(self, msg):
        self.latest_msg = msg
        self.save_msg(msg)

    def get_msg(self):
        return self.latest_msg
    
    # TODO: add record to DB
    def save_msg(self, msg_data):
        pass