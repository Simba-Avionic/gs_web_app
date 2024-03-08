from rclpy.node import Node
import json

class TopicReceiver(Node):
    def __init__(self, msg_config):
        super().__init__('number_subscriber')

        self.msg = json.loads(msg_config)

        self.subscription = self.create_subscription(
            self.msg["msg_type"], self.msg["topic_name"], 
            self.msg_callback, self.msg["interval"])
        
        self.latest_msg = None
        self.last_msg_timestamp = None

    def msg_callback(self, msg):
        self.latest_msg = msg.data