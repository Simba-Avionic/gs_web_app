from rclpy.node import Node
import gs_interfaces.msg
from rosidl_runtime_py import message_to_ordereddict
import json

class TopicReceiver(Node):
    def __init__(self, msg_type: str, topic_name: str, interval: int, msg_fields):
        super().__init__(msg_type)

        self.msg_fields = msg_fields

        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, msg_type), topic_name, self.msg_callback, interval)
        
        self.latest_msg = None
        self.last_msg_timestamp = None

    def msg_callback(self, msg):
        deserialized_msg = message_to_ordereddict(msg)
        self.latest_msg = json.dumps(deserialized_msg)

    def get_msg(self):
        return self.latest_msg