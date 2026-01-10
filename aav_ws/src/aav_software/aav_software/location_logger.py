#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class LocationLogger(Node):
    def __init__(self):
        super().__init__("location_logger")
        self.get_logger().info("Location Logger has been launched")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = LocationLogger()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
