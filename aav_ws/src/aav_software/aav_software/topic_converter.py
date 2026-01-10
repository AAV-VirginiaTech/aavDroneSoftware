#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter")
        self.get_logger().info("Topic Converter has been launched")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
