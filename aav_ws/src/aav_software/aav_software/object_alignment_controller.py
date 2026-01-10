#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class ObjectAlignmentController(Node):
    def __init__(self):
        super().__init__("object_alignment_controller")
        self.get_logger().info("Object Alignment Controller has been launched")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ObjectAlignmentController()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
