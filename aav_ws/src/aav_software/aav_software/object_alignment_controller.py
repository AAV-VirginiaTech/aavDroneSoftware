#!/usr/bin/env python3
from enum import Enum
from typing import Optional, cast

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition, TargetPosition
from rclpy.clock import Duration
from rclpy.node import Node

from .topic_converter_for_simulation import ArduPilotMode

# Finite State Machine Diagram
# https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764648320440275

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


class Mission(Enum):
    """
    Current mission being run by the Object Alignment Controller.
    Used to determine which parameters to use and which actions to take at various stages of the state machine.
    """

    PACKAGE_DELIVERY_CUASC = 0  # Mission for delivering cube onto the bullseye. Drone will land and take off autonomously.
    PAYLOAD_DROP_CUASC = 1  # Mission for dropping beanbag onto the bullseye. Drone remains in air the entire time.
    GCP_MARKER_ALIGNING_CUASC = (
        2  # Mission for drone only aligning to GCP points and not doing anything else.
    )
    PAYLOAD_DELIVERY_SUAS = 3  # Mission for delivering water bottle/strobe beacon to detected object on the ground. Drone stays in the air the entire time.


class OacState(Enum):
    """
    Implements the state machine diagram shown on the Object Alignment Controller Miro board
    """

    SEEKING = 0  # Monitoring for new targets; sending their positions while maintaining altitude
    ALIGNED_DESCENDING = 1  # Sending target positions while descending to alignment altitude; holds at alignment altitude for 8s
    FINAL_DESCENDING = 2  # Descending to drop altitude
    DROPPING_PAYLOAD = 3  # Waiting for payload drop to finish
    LANDING = 4  # In landing mode; waiting to reach ground
    DROPPING_PACKAGE = 5  # Waiting for package drop to finish
    TAKING_OFF = 6  # In takeoff mode; waiting to reach threshold altitude
    RETURNING = 7  # Back in AUTO mode; done


class ObjectAlignmentController(Node):
    SUBSTRUCTURE_ACTION_DURATION = Duration(seconds=5)
    SEEK_ALIGNMENT_DURATION = Duration(seconds=30)
    DESCENT_ALIGNMENT_DURATION = Duration(seconds=12)

    LANDING_THRESHOLD_ALTITUDE: float = 0.5
    TAKEOFF_THRESHOLD_ALTITUDE: float = 15.0

    def __init__(self):
        super().__init__("object_alignment_controller")

        self.mode_sub = self.create_subscription(
            Mode, "/AAV/current_mode", self.mode_callback, 10
        )
        self.target_position_sub = self.create_subscription(
            TargetPosition,
            "/AAV/estimated_target_position",
            self.target_position_callback,
            10,
        )
        self.current_drone_position_sub = self.create_subscription(
            DronePosition, "/AAV/current_gps_position", self.gps_position_callback, 10
        )

        self.new_position_pub = self.create_publisher(
            NewDronePosition, "/AAV/send_new_position", 10
        )
        self.new_mode_pub = self.create_publisher(Mode, "/AAV/set_mode", 10)

        self.state_machine_timer = self.create_timer(0.5, self.update_state_machine)

        self.current_mode = ArduPilotMode.AUTO
        self.guided_mode_request_in_flight = False
        self.current_gps_position: Optional[DronePosition] = None

        self.last_target_position: Optional[NewDronePosition] = None

        ### ROS2 PARAMETERS

        # Declare and get the package delivery mission parameter
        self.declare_parameter("current_mission", 0)
        self.current_mission = self.get_parameter("current_mission").value

        # Declare and get altitude parameters
        self.declare_parameter("descent_alignment_altitude", 5.0)
        self.descent_alignment_altitude = cast(
            float, self.get_parameter("descent_alignment_altitude").value
        )

        self.declare_parameter("hardcoded_drop_altitude", 3.0)
        self.hardcoded_drop_altitude = cast(
            float, self.get_parameter("hardcoded_drop_altitude").value
        )

        ###

        self.state = OacState.SEEKING
        self.time_marker = self.get_clock().now()

        self.get_logger().info("Object Alignment Controller has been launched")

    def send_new_mode(self, mode: ArduPilotMode):
        new_mode = Mode()
        new_mode.mode = mode
        self.new_mode_pub.publish(new_mode)

    def mode_callback(self, mode: Mode):
        self.current_mode = ArduPilotMode(mode.mode)

        # A mode update arrived; allow future mode requests if needed.
        self.guided_mode_request_in_flight = False

    def target_position_callback(self, target_position: TargetPosition):
        if self.state == OacState.RETURNING:
            return

        if self.state not in (OacState.SEEKING, OacState.ALIGNED_DESCENDING):
            return

        if self.current_mode != ArduPilotMode.GUIDED:
            if not self.guided_mode_request_in_flight:
                self.send_new_mode(ArduPilotMode.GUIDED)
                self.guided_mode_request_in_flight = True
            return

        if target_position.object_label != "Bullseye":
            self.get_logger().info("The detected object is not a Bullseye. Ignoring.")
            return

        # if this is the first target we have seen, reset the timer
        if not self.last_target_position:
            self.time_marker = self.get_clock().now()

        if self.state == OacState.SEEKING:
            if not self.current_gps_position:
                self.get_logger().warning(
                    "Current GPS position unavailable; cannot publish target position."
                )
                return

            new_position = NewDronePosition()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude
            new_position.altitude = float(self.current_gps_position.altitude)

            self.last_target_position = new_position
            self.new_position_pub.publish(new_position)
        elif self.state == OacState.ALIGNED_DESCENDING:
            new_position = NewDronePosition()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude

            new_position.altitude = self.descent_alignment_altitude
            self.last_target_position = new_position

            self.new_position_pub.publish(new_position)

    def gps_position_callback(self, gps_position: DronePosition):
        self.current_gps_position = gps_position

    def update_state_machine(self):
        match self.state:
            case OacState.SEEKING:
                if not self.last_target_position or not self.current_gps_position:
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    self.time_marker = self.get_clock().now()
                elif (
                    self.get_clock().now() - self.time_marker
                    > ObjectAlignmentController.SEEK_ALIGNMENT_DURATION
                ):
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending

                    self.state = OacState.ALIGNED_DESCENDING
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.ALIGNED_DESCENDING:
                if not self.current_gps_position:
                    self.time_marker = self.get_clock().now()
                else:
                    # within .25m of drop altitude, check timer
                    if (
                        abs(
                            float(self.current_gps_position.altitude)
                            - float(self.descent_alignment_altitude)
                        )
                        < 0.25
                    ):
                        # allow 8 seconds for final alignment
                        if (
                            self.get_clock().now() - self.time_marker
                            < ObjectAlignmentController.DESCENT_ALIGNMENT_DURATION
                        ):
                            pass
                        # if the timer has expired, move on to final descent
                        else:

                            self.state = OacState.FINAL_DESCENDING
                            self.get_logger().info(
                                f"Switching state to {self.state.name}"
                            )
                    # not within range of drop altitude, reset the timer
                    else:
                        self.time_marker = self.get_clock().now()

            case OacState.FINAL_DESCENDING:
                if self.current_gps_position and (
                    abs(
                        float(self.current_gps_position.altitude)
                        - float(self.hardcoded_drop_altitude)
                    )
                    < 0.25
                ):
                    # if at descent altitude, either start landing or run payload drop
                    if self.current_mission == Mission.PACKAGE_DELIVERY_CUASC.value:
                        self.send_new_mode(ArduPilotMode.LAND)
                        self.state = OacState.LANDING
                        self.get_logger().info(f"Switching state to {self.state.name}")
                    elif self.current_mission == Mission.PAYLOAD_DROP_CUASC.value:
                        # TODO: Call payload drop script
                        self.time_marker = self.get_clock().now()

                        self.state = OacState.DROPPING_PAYLOAD
                        self.get_logger().info(f"Switching state to {self.state.name}")
                else:
                    new_position = NewDronePosition()
                    new_position.longitude = self.current_gps_position.longitude
                    new_position.latitude = self.current_gps_position.latitude
                    new_position.altitude = self.hardcoded_drop_altitude
                    self.new_position_pub.publish(new_position)

            case OacState.DROPPING_PAYLOAD:
                if (
                    self.get_clock().now() - self.time_marker
                    > ObjectAlignmentController.SUBSTRUCTURE_ACTION_DURATION
                ):
                    self.send_new_mode(ArduPilotMode.AUTO)

                    self.state = OacState.RETURNING
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.LANDING:
                if self.current_gps_position and float(
                    self.current_gps_position.altitude
                ) < float(ObjectAlignmentController.LANDING_THRESHOLD_ALTITUDE):
                    # TODO: Call package drop script
                    self.time_marker = self.get_clock().now()
                    self.state = OacState.DROPPING_PACKAGE
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.DROPPING_PACKAGE:
                if (
                    self.get_clock().now() - self.time_marker
                    > ObjectAlignmentController.SUBSTRUCTURE_ACTION_DURATION
                ):
                    self.send_new_mode(ArduPilotMode.TAKEOFF)

                    self.state = OacState.TAKING_OFF
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.TAKING_OFF:
                if self.current_gps_position and float(
                    self.current_gps_position.altitude
                ) > float(ObjectAlignmentController.TAKEOFF_THRESHOLD_ALTITUDE):
                    self.send_new_mode(ArduPilotMode.AUTO)

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
