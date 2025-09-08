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

MSG_DEFS = {}

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
        val_name = field['val_name']
        msg_def_name = field.get('msg_def')

        if msg_def_name:
            parts = val_name.split('/') # 0 - v7_4, 1 - bus_voltage_v

            nested_msg_type = import_message_type(msg_def_name)
            nested_msg = nested_msg_type()
            populate_message_fields(
                nested_msg, MSG_DEFS[msg_def_name], stamp, msg_def_name
            )
            setattr(msg, parts[0], nested_msg)
            continue

        if '/' in val_name:
            val_name = val_name.split('/')[-1]

        if field_type == 'std_msgs/Header':
            msg.header.stamp = stamp
            msg.header.frame_id = msg_type_name
        elif field_type in ('float32', 'float64', 'float'):
            if val_name == 'lat':
                setattr(msg, val_name, random.uniform(54.20, 54.45))
            elif val_name == 'lon':
                setattr(msg, val_name, random.uniform(18.50, 18.75))
            elif val_name == 'combined_fuel_kg':
                setattr(msg, val_name, random.uniform(0, 20))
            elif 'kg' in val_name:
                setattr(msg, val_name, random.uniform(0, 30))
            else:
                setattr(msg, val_name, random.uniform(-100.0, 100.0))
        elif field_type in ('int8', 'uint8', 'uint32', 'int32', 'uint32', 'uint64', 'int', 'int16', 'uint16','int64'):
            if 'status' in val_name:
                setattr(msg, val_name, random.randint(0, 7))
            elif val_name == 'state':
                setattr(msg, val_name, random.randint(0, 5))
            elif 'kg' in val_name:
                setattr(msg, val_name, random.randint(0, 100))
            else:
                setattr(msg, val_name, random.randint(0, 100))
        elif field_type == 'bool':
            setattr(msg, val_name, random.choice([True, False]))
        else:
            print(f'Unknown field type: {field_type} for field {val_name}')


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
    
    for msg_def in config['msg_defs']:
        MSG_DEFS[msg_def['msg_type']] = msg_def['msg_fields']

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
