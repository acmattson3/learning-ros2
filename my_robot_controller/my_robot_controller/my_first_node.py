#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self._counter = 0
        self.create_timer(1.0, self.timer_callback) # Call timer_callback every second
    
    def timer_callback(self):
        self.get_logger().info(f"Hello from ROS2: {self._counter}")
        self._counter += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = MyNode()
    rclpy.spin(node) # Keep the node alive
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()