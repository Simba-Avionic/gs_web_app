#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import threading

class IntegerPublisher(Node):
    def __init__(self):
        super().__init__('number')
        self.publisher_ = self.create_publisher(Int32, 'number', 10)
        self.timer_ = self.create_timer(1, self.publish_number)
        self.number_ = 0

    def publish_number(self):
        msg = Int32()
        msg.data = self.number_
        self.publisher_.publish(msg)
        self.number_ += 1

class FloatPublisher(Node):
    def __init__(self):
        super().__init__('float')
        self.publisher_ = self.create_publisher(Float32, 'float', 10)
        self.timer_ = self.create_timer(1, self.publish_number)
        self.number_ = 0

    def publish_number(self):
        msg = Float32()
        msg.data = self.number_
        self.publisher_.publish(msg)
        self.number_ += 0.33

def main(args=None):
    rclpy.init(args=args)

    node = IntegerPublisher()
    node2 = FloatPublisher()

    thread2 = threading.Thread(target=rclpy.spin, args=(node2,), daemon=True)
    thread1 = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()