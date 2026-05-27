#!/usr/bin/env python3
from enum import Enum
from typing import Optional, cast

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition, TargetPosition
from rclpy.clock import Duration
from rclpy.node import Node
from rclpy.time import Time

from .mission import Mission
from .payload_drop import run_payload_drop_sequence
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
    STARTUP_DELAY = Duration(seconds=45)
    SEEK_ALIGNMENT_DURATION = Duration(seconds=30)
    DESCENT_ALIGNMENT_DURATION = Duration(seconds=12)

    LANDING_THRESHOLD_ALTITUDE: float = 0.5
    TAKEOFF_THRESHOLD_ALTITUDE: float = 3.0

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

        self.last_target_label: Optional[str] = None
        self.seen_target: bool = False

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

        # Declare and get the substructure action duration mission parameter
        self.declare_parameter("substructure_action_duration", 5)
        self.substructure_action_duration = Duration(
            seconds=cast(int, self.get_parameter("substructure_action_duration").value)
        )

        ###

        self.state = OacState.SEEKING
        self.time_marker = self.get_clock().now()
        self.startup_time: Optional[Time] = None
        self.startup_delay_ended = False

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
        # Don't process targets until the drone has taken off and the startup delay has elapsed
        if self.startup_time is None or (
            self.startup_time is not None
            and (
                self.get_clock().now() - self.startup_time
                < ObjectAlignmentController.STARTUP_DELAY
            )
        ):
            return

        if self.state == OacState.RETURNING:
            return

        if self.state not in (OacState.SEEKING, OacState.ALIGNED_DESCENDING):
            return

        if (
            self.current_mode != ArduPilotMode.GUIDED
            and self.current_mission != Mission.PAYLOAD_DELIVERY_SUAS.value
            and self.current_mission != Mission.GCP_MARKER_ALIGNING_CUASC.value
        ):
            if not self.guided_mode_request_in_flight:
                self.send_new_mode(ArduPilotMode.GUIDED)
                self.guided_mode_request_in_flight = True
            return

        if self.current_mode != ArduPilotMode.GUIDED:
            return

        self.last_target_label = target_position.object_label

        # if this is the first target we have seen, reset the timer
        if not self.seen_target:
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

            self.seen_target = True

            self.new_position_pub.publish(new_position)
        elif self.state == OacState.ALIGNED_DESCENDING:
            new_position = NewDronePosition()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude

            new_position.altitude = self.descent_alignment_altitude

            self.new_position_pub.publish(new_position)

    def gps_position_callback(self, gps_position: DronePosition):
        self.current_gps_position = gps_position

    def update_state_machine(self):
        # Start the startup delay timer once drone takes off
        if (
            self.startup_time is None
            and self.current_gps_position
            and float(self.current_gps_position.altitude)
            > float(ObjectAlignmentController.TAKEOFF_THRESHOLD_ALTITUDE)
        ):
            self.startup_time = self.get_clock().now()
            self.get_logger().info("Drone has taken off; starting startup delay")

        # Wait for startup delay before processing state transitions
        if self.startup_time is not None and (
            self.get_clock().now() - self.startup_time
            < ObjectAlignmentController.STARTUP_DELAY
        ):
            self.get_logger().info("Under startup delay")
            return

        # Log when startup delay ends
        if self.startup_time is not None and not self.startup_delay_ended:
            self.startup_delay_ended = True
            self.get_logger().info("Startup delay has ended")

        match self.state:
            case OacState.SEEKING:
                if (
                    not self.seen_target
                    or not self.current_gps_position
                    or self.current_mission == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
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
                        self.get_logger().info("Dropping payload")
                        try:
                            run_payload_drop_sequence(self, True, 2000, 8)
                        except Exception as exc:
                            self.get_logger().error(
                                f"Payload drop sequence failed: {exc}"
                            )
                        self.time_marker = self.get_clock().now()

                        self.state = OacState.DROPPING_PAYLOAD
                        self.get_logger().info(f"Switching state to {self.state.name}")
                    elif self.current_mission == Mission.PAYLOAD_DELIVERY_SUAS.value:
                        self.time_marker = self.get_clock().now()
                        if self.last_target_label == "manikin":
                            self.get_logger().info("Dropping water bottle")

                            try:
                                run_payload_drop_sequence(self, True, 2000, 7)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )
                        else:
                            self.get_logger().info("Dropping beacon")
                            try:
                                run_payload_drop_sequence(self, True, 2000, 6)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )

                        self.state = OacState.DROPPING_PAYLOAD
                        self.get_logger().info(f"Switching state to {self.state.name}")

                else:
                    new_position = NewDronePosition()
                    if self.current_gps_position:
                        new_position.longitude = self.current_gps_position.longitude
                        new_position.latitude = self.current_gps_position.latitude
                        new_position.altitude = self.hardcoded_drop_altitude
                        self.new_position_pub.publish(new_position)

            case OacState.DROPPING_PAYLOAD:
                if (
                    self.get_clock().now() - self.time_marker
                    > self.substructure_action_duration
                ):
                    if self.current_mission == Mission.PAYLOAD_DELIVERY_SUAS.value:
                        self.seen_target = False
                        self.state = OacState.SEEKING
                        self.get_logger().info(f"Switching state to {self.state.name}")
                    else:
                        self.send_new_mode(ArduPilotMode.AUTO)
                        self.state = OacState.RETURNING
                        self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.LANDING:
                if self.current_gps_position and float(
                    self.current_gps_position.altitude
                ) < float(ObjectAlignmentController.LANDING_THRESHOLD_ALTITUDE):
                    self.get_logger().info("Dropping CUASC package")
                    try:
                        run_payload_drop_sequence(self, False, 1500, 8)
                    except Exception as exc:
                        self.get_logger().error(f"Payload drop sequence failed: {exc}")

                    self.time_marker = self.get_clock().now()
                    self.state = OacState.DROPPING_PACKAGE
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.DROPPING_PACKAGE:
                if (
                    self.get_clock().now() - self.time_marker
                    > self.substructure_action_duration
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
