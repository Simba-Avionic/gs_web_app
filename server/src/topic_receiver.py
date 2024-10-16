from rclpy.node import Node
import gs_interfaces.msg
from rosidl_runtime_py import message_to_ordereddict
import threading
class TopicReceiver(Node):
    def __init__(self, msg_type: str, topic_name: str, interval: int, msg_fields):
        super().__init__(msg_type)

        self.msg_fields = msg_fields
        self._lock = threading.Lock()
        self.curr_msg = None
        self.curr_msg_db = None

        self.subscription = self.create_subscription(
            getattr(gs_interfaces.msg, msg_type), topic_name, self.msg_callback, 10)

    def msg_callback(self, msg):
        msg = message_to_ordereddict(msg)
        self.curr_msg = msg
        self.curr_msg_db = msg

    def get_msg(self):
        msg = self.curr_msg
        if msg is not None:
            self.curr_msg = None
        return msg
    
    def get_msg_db(self):
        msg = self.curr_msg_db
        if msg is not None:
            self.curr_msg_db = None
            return msg