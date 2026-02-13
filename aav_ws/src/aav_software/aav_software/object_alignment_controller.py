#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aav_msgs.msg import Mode
from aav_msgs.msg import TargetPosition
from aav_msgs.msg import LatLong
from .topic_converter import ArduPilotMode

# TODO Need to send command to land drone. Just need to change ardupilot mode to LAND. 
# ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"

# TODO Need to automatically drop payload (call electrical python code)


"""
# Launch Object Alignment Controller
ros2 run aav_software object_alignment_controller

#View output topic
ros2 topic echo /AAV/send_new_position

# Send new mode

ros2 topic pub --once /AAV/current_mode aav_msgs/msg/Mode "{mode: 4}"

# Send new target position

ros2 topic pub --once /AAV/estimated_target_position aav_msgs/msg/TargetPosition "{
object_label: 'target_1',
latitude: 37.2296,
longitude: -80.4139
  }"


# Test landing drone
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"

"""


class ObjectAlignmentController(Node):
    def __init__(self):
        super().__init__("object_alignment_controller")

        self.modeSub = self.create_subscription(Mode, "/AAV/current_mode", self.callback_mode, 10)
        self.posSub = self.create_subscription(TargetPosition, "/AAV/estimated_target_position", self.callback_pos, 10)
        self.posPub = self.create_publisher(LatLong, "/AAV/send_new_position", 10)

        self.currentMode = ArduPilotMode.AUTO
        self.lastObjectLabel = "none"

        self.get_logger().info("Object Alignment Controller has been launched")

    def callback_mode(self, current_mode: Mode):
        self.get_logger().info(f"Recieved new mode: {current_mode}")
        self.currentMode = ArduPilotMode(current_mode.mode)

    def callback_pos(self, target_position: TargetPosition):
        self.get_logger().info(f"Recieved new target position: {target_position}")
        if (self.currentMode != ArduPilotMode.GUIDED):
            self.get_logger().info("Not guided. Doing nothing.")
            return

        self.get_logger().info(f"Sending new positition")
        self.lastObjectLabel = target_position.object_label

        new_position = LatLong()
        new_position.latitude = target_position.latitude
        new_position.longitude = target_position.longitude
        self.posPub.publish(new_position)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAlignmentController()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
