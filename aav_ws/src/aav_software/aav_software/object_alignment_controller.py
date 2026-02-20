#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
from aav_msgs.msg import Mode
from aav_msgs.msg import TargetPosition
from aav_msgs.msg import LatLong
from aav_msgs.msg import DronePosition
from .topic_converter import ArduPilotMode
from enum import Enum

# TODO: Implement mode switching todos below
# TODO: Replace magic numbers with named constants
# TODO: Interface with substructure code

# Testing commands:
"""
# Launch Object Alignment Controller
ros2 run aav_software object_alignment_controller

#View output topic
ros2 topic echo /AAV/send_new_position

ros2 topic echo /AAV/set_mode

# Send new mode

ros2 topic pub --once /AAV/current_mode aav_msgs/msg/Mode "{mode: 4}"

# Send new target position

ros2 topic pub --once /AAV/estimated_target_position aav_msgs/msg/TargetPosition "{
object_label: 'target_1',
latitude: 37.2296,
longitude: -80.4139
  }"
  
  
# Send new drone position
ros2 topic pub --once /AAV/current_gps_position aav_msgs/msg/DronePosition "{
latitude: 37.2295,
longitude: -80.4138,
altitude: 30.0,
yaw: 0.0
}"



"""

class OacState(Enum):
    SEEKING = 0             # Monitoring for new targets; sending their positions
    DROPPING_PAYLOAD = 1    # Waiting for payload drop to finish
    LANDING = 2             # In landing mode; waiting to reach ground
    DROPPING_PACKAGE = 3    # Waiting for package drop to finish
    TAKING_OFF = 4          # In takeoff mode; waiting to reach threshold altitude
    RETURNING = 5           # In RTL mode; done


class ObjectAlignmentController(Node):
    def __init__(self):
        super().__init__("object_alignment_controller")

        self.modeSub = self.create_subscription(Mode, "/AAV/current_mode", self.mode_callback, 10)
        self.targetPositionSub = self.create_subscription(TargetPosition, "/AAV/estimated_target_position", self.target_position_callback, 10)
        self.currentDronePositionSub = self.create_subscription(DronePosition, "AAV/current_gps_position", self.gps_position_callback, 10)
        self.newPositionPub = self.create_publisher(LatLong, "/AAV/send_new_position", 10)

        self.stateMachineTimer = self.create_timer(0.5, self.update_state_machine)

        self.currentMode = ArduPilotMode.AUTO
        self.currentGpsPosition = None

        self.lastTargetPosition = None

        self.doingPackageDeliveryMission = True

        self.state = OacState.SEEKING
        self.timeMarker = self.get_clock().now()

        self.get_logger().info("Object Alignment Controller has been launched")

    def mode_callback(self, current_mode: Mode):
        self.get_logger().info(f"Recieved new mode: {current_mode}")
        self.currentMode = ArduPilotMode(current_mode.mode)

    def target_position_callback(self, target_position: TargetPosition):
        self.get_logger().info(f"Recieved new target position: {target_position}")
        if (self.currentMode != ArduPilotMode.GUIDED):
            self.get_logger().info("Not guided. Doing nothing.")
            return

        if (target_position.object_label != "Bullseye"):
            self.get_logger().info("The detected object is not a bullseye. Ignoring.")
            return

        if (self.state == OacState.SEEKING):
            # TODO: Switch mode to guided

            new_position = LatLong()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude

            self.lastTargetPosition = new_position

            self.newPositionPub.publish(new_position)

    def gps_position_callback(self, gps_position: DronePosition):
        self.currentGpsPosition = gps_position

    def update_state_machine(self):
        match self.state:
            case OacState.SEEKING:
                if self.lastTargetPosition == None or self.currentGpsPosition.altitude > 8:
                    pass # keep seeking
                elif self.doingPackageDeliveryMission:
                    # TODO: Switch mode to land
                    self.state = OacState.LANDING
                else:
                    # TODO: Call payload drop script
                    self.timeMarker = self.get_clock().now()
                    self.state = OacState.DROPPING_PAYLOAD

            case OacState.DROPPING_PAYLOAD:
                if (self.get_clock().now() - self.timeMarker > Duration(seconds=5)):
                    # TODO: Switch mode to RTL
                    self.state = OacState.RETURNING

            case OacState.LANDING:
                if self.currentGpsPosition.altitude < 0.5:
                    # TODO: Call package drop script
                    self.timeMarker = self.get_clock().now()
                    self.state = OacState.DROPPING_PACKAGE

            case OacState.DROPPING_PACKAGE:
                if (self.get_clock().now() - self.timeMarker > Duration(seconds=5)):
                    # TODO: Switch mode to takeoff
                    self.state = OacState.TAKING_OFF

            case OacState.TAKING_OFF:
                if self.currentGpsPosition.altitude > 30:
                    # TODO: Switch mode to RTL
                    self.state = OacState.RETURNING

            case OacState.RETURNING:
                # Nothing to do for now
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAlignmentController()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
