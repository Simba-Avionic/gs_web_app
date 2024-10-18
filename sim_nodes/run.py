import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import json
import os
import threading
import gs_interfaces.msg
import random

def load_main_config(path_to_conf):
    config = None
    with open(path_to_conf, "r") as f:
        config = json.load(f)
        return config

def import_message_type(msg_type):
    return getattr(gs_interfaces.msg, msg_type)

def spin_node(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()

def get_slots_and_types_cleaned(ros_msg):
    if hasattr(ros_msg, '__slots__') and hasattr(ros_msg, '__slot_types__'):
        # Remove the underscore prefix from each slot
        cleaned_slots = [slot.lstrip('_') for slot in ros_msg.__slots__]
        slot_types = ros_msg.__slot_types__
        return list(zip(cleaned_slots, slot_types))  # Pair cleaned slots with their types
    else:
        return None

def populate_message_fields(msg, field_config, stamp, msg_type_name):
    """
    Populate the fields of the given message based on the field configuration.
    """
    for field in field_config:
        field_type = field['type']
        field_name = field['val_name']

        if field_type == 'std_msgs/Header':
            msg.header.stamp = stamp
            msg.header.frame_id = msg_type_name
        elif field_type in ('float32', 'float64', 'float'):
            setattr(msg, field_name, random.uniform(-100.0, 100.0))
        elif field_type in ('int8', 'int32', 'uint32', 'int'):
            setattr(msg, field_name, random.randint(0, 100))
        elif field_type == 'bool':
            setattr(msg, field_name, True)
        else:
            # Dynamically import and set nested message types
            package_name, message_name = field_type.split('/')
            nested_msg_type = import_message_type(message_name)
            nested_msg = nested_msg_type()

            if hasattr(nested_msg, '__slots__'):
                data = nested_msg_type.get_fields_and_field_types()
                msg_fields = [
                    {'val_name': key, 'type': value} 
                    for key, value in data.items()
                ]
                populate_message_fields(
                    nested_msg, msg_fields, stamp, msg_type_name
                )
            setattr(msg, field_name, nested_msg)

class DynamicNode(Node):
    def __init__(self, config):
        super().__init__(config['msg_type'])
        
        self.msg_fields = config['msg_fields']
        self.topic_name = config['topic_name']
        self.interval = config['interval'] / 1000.0  # Convert ms to seconds
        self.msg_type_name = config['msg_type']
        self.msg_type = import_message_type(self.msg_type_name)
        self.publisher = self.create_publisher(self.msg_type, self.topic_name, 10)
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def build_message(self):
        msg = self.msg_type()
        populate_message_fields(
            msg, self.msg_fields, self.get_clock().now().to_msg(), self.msg_type_name
        )
        return msg

    def timer_callback(self):
        msg = self.build_message()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published message on {self.topic_name}')

def main():
    rclpy.init()

    curr_file_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(curr_file_dir, "../config.json")
    config = load_main_config(config_path)

    node_threads = []
    for node_config in config['topics']:
        node = DynamicNode(node_config)
        node_thread = threading.Thread(target=spin_node, args=(node,))
        node_threads.append(node_thread)  

    for thread in node_threads:
        thread.start()
        time.sleep(0.1)

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        for thread in node_threads:
            thread.join()

if __name__ == '__main__':
    main()
