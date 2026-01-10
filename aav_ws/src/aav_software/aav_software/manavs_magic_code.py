#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class ManavsMagicCode(Node):
    def __init__(self):
        super().__init__("manavs_magic_code")
        self.get_logger().info("Manav's Magic Code has been launched")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ManavsMagicCode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
