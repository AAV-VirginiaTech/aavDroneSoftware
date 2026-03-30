#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
from aav_msgs.msg import Mode
from aav_msgs.msg import TargetPosition
from aav_msgs.msg import NewDronePosition
from aav_msgs.msg import DronePosition
from .topic_converter_for_simulation import ArduPilotMode
from enum import Enum

# Reference Pseudocode:
# https://www.notion.so/vtaav/Object-Alignment-Controller-Pseudocode-308623fcf7fe80609d81f3410f0f6a13?source=copy_link

# TODO: Interface with substructure control code

# TODO: (CARTER) Need to change from sending RTL mode to sending AUTO mode


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
    """
    Impliments the state machine diagram shown on the Object Alignment Controller Miro board
    """
    SEEKING = 0             # Monitoring for new targets; sending their positions
    DROPPING_PAYLOAD = 1    # Waiting for payload drop to finish
    LANDING = 2             # In landing mode; waiting to reach ground
    DROPPING_PACKAGE = 3    # Waiting for package drop to finish
    TAKING_OFF = 4          # In takeoff mode; waiting to reach threshold altitude
    RETURNING = 5           # In RTL mode; done


class ObjectAlignmentController(Node):
    PROXIMITY_THRESHOLD_ALTITUDE = 8.0
    SUBSTRUCTURE_ACTION_DURATION = Duration(seconds=5)
    LANDING_THRESHOLD_ALTITUDE = 0.5
    TAKEOFF_THRESHOLD_ALTITUDE = 30
    HARDCODED_DROP_ALTITUDE = 3.0

    def __init__(self):
        super().__init__("object_alignment_controller")

        self.mode_sub = self.create_subscription(Mode, "/AAV/current_mode", self.mode_callback, 10)
        self.target_position_sub = self.create_subscription(TargetPosition, "/AAV/estimated_target_position", self.target_position_callback, 10)
        self.current_drone_position_sub = self.create_subscription(DronePosition, "AAV/current_gps_position", self.gps_position_callback, 10)

        self.new_position_pub = self.create_publisher(NewDronePosition, "/AAV/send_new_position", 10)
        self.new_mode_pub = self.create_publisher(Mode, "/AAV/set_mode", 10)

        self.state_machine_timer = self.create_timer(0.5, self.update_state_machine)

        self.current_mode = ArduPilotMode.AUTO
        self.current_gps_position = None

        self.last_target_position = None

        self.doing_package_delivery_mission = True

        self.state = OacState.SEEKING
        self.time_marker = self.get_clock().now()

        self.get_logger().info("Object Alignment Controller has been launched")

    def send_new_mode(self, mode: ArduPilotMode):
        new_mode = Mode()
        new_mode.mode = mode
        self.new_mode_pub.publish(new_mode)


    def mode_callback(self, mode: Mode):
        self.get_logger().info(f"Recieved new mode: {ArduPilotMode(mode.mode).name}")
        self.current_mode = ArduPilotMode(mode.mode)

    def target_position_callback(self, target_position: TargetPosition):
        self.get_logger().info(f"Recieved new target position: {target_position}")
        if (self.current_mode != ArduPilotMode.GUIDED):
            # TODO: (CARTER) Need to change to guided mode if not in AUTO mode when we detect something new
            self.get_logger().info("Not guided. Doing nothing.")
            return

        if (target_position.object_label != "bullseye"):
            self.get_logger().info("The detected object is not a bullseye. Ignoring.")
            return

        if (self.state == OacState.SEEKING):
            new_position = NewDronePosition()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude
            new_position.altitude = ObjectAlignmentController.HARDCODED_DROP_ALTITUDE

            self.last_target_position = new_position

            self.new_position_pub.publish(new_position)

    def gps_position_callback(self, gps_position: DronePosition):
        self.get_logger().info(f"Recieved new gps position: {gps_position}")
        self.current_gps_position = gps_position

    def update_state_machine(self):
        match self.state:
            case OacState.SEEKING:
                if (not self.last_target_position or
                    not self.current_gps_position or
                    self.current_gps_position.altitude > ObjectAlignmentController.PROXIMITY_THRESHOLD_ALTITUDE):
                    # TODO: Add more detailed proximity checks
                    # keep seeking
                    pass
                elif self.doing_package_delivery_mission:
                    self.send_new_mode(ArduPilotMode.LAND)

                    self.state = OacState.LANDING
                    self.get_logger().info(f"Switching mode to {self.state.name}")
                else:
                    # TODO: Call payload drop script
                    self.time_marker = self.get_clock().now()

                    self.state = OacState.DROPPING_PAYLOAD
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.DROPPING_PAYLOAD:
                if self.get_clock().now() - self.time_marker > ObjectAlignmentController.SUBSTRUCTURE_ACTION_DURATION:
                    self.send_new_mode(ArduPilotMode.RTL)

                    self.state = OacState.RETURNING
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.LANDING:
                if self.current_gps_position and self.current_gps_position.altitude < ObjectAlignmentController.LANDING_THRESHOLD_ALTITUDE:
                    # TODO: Call package drop script
                    self.time_marker = self.get_clock().now()
                    self.state = OacState.DROPPING_PACKAGE
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.DROPPING_PACKAGE:
                if self.get_clock().now() - self.time_marker > ObjectAlignmentController.SUBSTRUCTURE_ACTION_DURATION:
                    self.send_new_mode(ArduPilotMode.TAKEOFF)

                    self.state = OacState.TAKING_OFF
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.TAKING_OFF:
                if self.current_gps_position and self.current_gps_position.altitude > ObjectAlignmentController.TAKEOFF_THRESHOLD_ALTITUDE:
                    self.send_new_mode(ArduPilotMode.RTL)

                    self.state = OacState.RETURNING
                    self.get_logger().info(f"Switching state to {self.state.name}")

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
