"""
Script for simulating ROS2 nodes.
"""

import os
import sys
import time
import json
import rclpy
import random
import threading
import gs_interfaces.msg
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))
from shared.paths import CONFIG_JSON_PATH

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

def populate_message_fields(msg, field_config, stamp, msg_type_name):
    """
    Populate the fields of the given message based on the field configuration in config.json.
    """
    for field in field_config:
        field_type = field['type']
        field_name = field['val_name']

        if field_type == 'std_msgs/Header':
            msg.header.stamp = stamp
            msg.header.frame_id = msg_type_name
        elif field_type in ('float32', 'float64', 'float'):
            if field_name == 'latitude':
                setattr(msg, field_name, random.uniform(54.20, 54.45))
            elif field_name == 'longitude':
                setattr(msg, field_name, random.uniform(18.50, 18.75))
            elif field_name == 'combined_fuel_kg':
                setattr(msg, field_name, random.uniform(0, 20))
            else:
                setattr(msg, field_name, random.uniform(-100.0, 100.0))
        elif field_type in ('int8', 'uint8', 'uint32', 'int32', 'uint32', 'uint64', 'int', 'int16', 'uint16','int64'):
            if 'status' in field_name:
                setattr(msg, field_name, random.randint(0, 7))
            else:
                setattr(msg, field_name, random.randint(0, 100))
        elif field_type == 'bool':
            setattr(msg, field_name, random.choice([True, False]))
        else:
            # Dynamically import and set nested message types
            _, message_name = field_type.split('/')
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

    config = load_main_config(CONFIG_JSON_PATH)

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
