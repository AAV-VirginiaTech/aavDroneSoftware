#!/usr/bin/env python3
from __future__ import annotations

from typing import cast

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition, TargetPosition
from rclpy.clock import Duration
from rclpy.node import Node

from aav_ws.build.aav_software.build.lib.aav_software.guidance import OacState

from .mission import Mission
from .payload_drop import run_payload_drop_sequence
from .topic_converter_for_simulation import ArduPilotMode
from .util.statemachine import StateMachine
from .util.timer import Timer

# Finite State Machine Diagram
# https://miro.com/app/board/uXjVIjMwI14=/?focusWidget=3458764648320440275

# Testing commands:
"""
# Launch Guidance2 System
ros2 run aav_software Guidance2

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


class Guidance2(Node):
    class State:
        """
        "Guidance refers to the determination of the desired path of travel (the
        "trajectory") from the vehicle's current location to a designated target,
        as well as desired changes in velocity, rotation and acceleration for
        following that path." - Wikipedia
        Implements the state machine diagram shown on the Guidance2 Miro board
        """

        class Base(StateMachine.State):
            def __init__(self, name: str, guidance: Guidance2):
                super().__init__(name)
                self.guidance = guidance

            def get_guidance(self) -> Guidance2:
                return self.guidance

            def get_logger(self):
                return self.guidance.get_logger()

            def log_throttled(self, level: str, key: str, message: str):
                self.guidance._log_throttled(level, key, message)

            def current_gps_position(self) -> DronePosition | None:
                return self.guidance.current_gps_position

            def current_mission(self):
                return self.guidance.current_mission

            def timer(self) -> Timer:
                return self.guidance.timer

            def descent_alignment_altitude(self):
                return self.guidance.descent_alignment_altitude
            
            def hardcoded_drop_altitude(self):
                return self.guidance.hardcoded_drop_altitude

            def send_new_mode(self,mode: ArduPilotMode):
                self.guidance.send_new_mode(mode)

            def run_payload_drop_sequence(
                self,
                openandclose: bool,
                pwm: float,
                aux_port: int,
            ) -> None:
                run_payload_drop_sequence(
                    self.guidance, openandclose, pwm, aux_port
                )

            def last_target_label(self) -> str | None:
                return self.guidance.last_target_label

            def new_position_pub(self):
                return self.guidance.new_position_pub

        class WaitForStartup(Base):
            """
            Waiting for the drone to take off and for the startup delay to elapse
            """

            DELAY = Duration(seconds=45)

            def __init__(self, guidance: Guidance2):
                super().__init__("wait_for_startup", guidance)
                self.wait_started = False

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered wait for startup mode; "
                    f"Waiting for drone to take off and for {self.DELAY.nanoseconds / 1e9}s startup delay to elapse"
                )

            def update(self):
                gps_pos = super().current_gps_position()
                if not gps_pos:
                    return

                current_altitude = float(gps_pos.altitude)
                threshold_altitude = float(Guidance2.TAKEOFF_THRESHOLD_ALTITUDE)

                # Start the startup delay timer once drone takes off
                if not self.wait_started and current_altitude > threshold_altitude:
                    super().timer().reset()
                    super().get_logger().info(
                        f"Drone has taken off at altitude {gps_pos.altitude}m; "
                        f"starting {self.DELAY.nanoseconds / 1e9}s startup delay"
                    )
                    self.wait_started = True

                # check if we've waited longer than the startup delay
                waited = super().timer().elapsed()
                if waited > self.DELAY:
                    # initiate seeking mode
                    super().get_guidance().startup_delay_ended = True
                    super().get_logger().info(
                        "Startup delay has ended; GCP marker tracking is now active"
                    )
                    self.mark_finished(Guidance2.State.Seeking(self.guidance))
                    return

                # debug waiting state
                current_mission = super().current_mission
                goal_mission = Mission.GCP_MARKER_ALIGNING_CUASC.value
                if current_mission == goal_mission:
                    super().log_throttled(
                        "info",
                        "gcp_startup_delay_progress",
                        f"GCP Mission: Startup delay in progress ({waited:.1f}s / "
                        f"{self.DELAY.nanoseconds / 1e9}s)",
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting wait for startup mode; Guidance2 system is now active"
                )

        class Seeking(Base):
            """
            Monitoring for new targets; sending their positions while maintaining altitude
            """

            DURATION = Duration(seconds=30)

            def __init__(self, guidance: Guidance2):
                super().__init__("seeking", guidance)
                self.guidance: Guidance2 = guidance
                self.seen_target: bool = False

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = Guidance2.State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class AlignedDescending(Base):
            """
            Sending target positions while descending to alignment altitude; holds at alignment altitude for 8s
            """

            DURATION = Duration(seconds=20)

            def __init__(self, guidance: Guidance2):
                super().__init__("aligned_descending", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                gps_pos = super().current_gps_position()
                if not gps_pos:
                    super().timer().reset()
                    return
                
                # within .25m of drop altitude, check timer
                altitude_dif = abs(gps_pos.altitude - super().descent_alignment_altitude())
                if altitude_dif < 0.25:
                    # allow 8 seconds for final alignment
                    if super().timer().elapsed() < self.DURATION:
                        pass
                    # if the timer has expired, move on to final descent
                    else:
                        self.state = OacState.FINAL_DESCENDING
                        super().get_logger().info(
                            f"Switching state to {self.state.name}"
                        )
                # not within range of drop altitude, reset the timer
                else:
                    super().timer().reset()

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class FinalDescending(Base):
            """
            Descending to drop altitude
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("final_descending", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Descending to drop altitude: {super().hardcoded_drop_altitude()}m"
                )

            def update(self):
                gps_pos = super().current_gps_position()
                if not gps_pos:
                    return

                # if within .25m of drop alititude
                if abs(gps_pos.altitude - super().hardcoded_drop_altitude()) < 0.25:
                    
                    # if at descent altitude, either start landing or run payload drop
                    if self.current_mission == Mission.PACKAGE_DELIVERY_CUASC.value:
                        super().send_new_mode(ArduPilotMode.LAND)
                        self.state = OacState.LANDING
                        self.get_logger().info(f"Switching state to {self.state.name}")
                    elif self.current_mission == Mission.PAYLOAD_DROP_CUASC.value:
                        self.get_logger().info("Dropping payload")
                        try:
                            super().run_payload_drop_sequence(True, 2000, 8)
                        except Exception as exc:
                            self.get_logger().error(
                                f"Payload drop sequence failed: {exc}"
                            )
                        super().timer().reset()

                        self.state = OacState.DROPPING_PAYLOAD
                        self.get_logger().info(f"Switching state to {self.state.name}")
                    elif self.current_mission == Mission.PAYLOAD_DELIVERY_SUAS.value:
                        super().timer().reset()
                        if self.last_target_label == "manikin":
                            self.get_logger().info("Dropping water bottle")

                            try:
                                super().run_payload_drop_sequence(True, 2000, 7)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )
                        else:
                            self.get_logger().info("Dropping beacon")
                            try:
                                super().run_payload_drop_sequence(True, 2000, 6)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )

                        self.state = OacState.DROPPING_PAYLOAD
                        self.get_logger().info(f"Switching state to {self.state.name}")

                else:
                    new_position = NewDronePosition()

                    gps_pos = super().current_gps_position()
                    if gps_pos:
                        new_position.longitude = gps_pos.longitude
                        new_position.latitude = gps_pos.latitude
                        new_position.altitude = super().hardcoded_drop_altitude()
                        self.new_position_pub().publish(new_position)

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class DroppingPayload(Base):
            """
            Waiting for payload drop to finish
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("dropping_payload", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = super().State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class Landing(Base):
            """
            In landing mode; waiting to reach ground
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("landing", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = super().State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class DroppingPackage(Base):
            """
            Waiting for package drop to finish
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("dropping_package", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = super().State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class TakingOff(Base):
            """
            In takeoff mode; waiting to reach threshold altitude
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("taking_off", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = super().State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

        class Returning(Base):
            """
            Back in AUTO mode; done
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("returning", guidance)

            def enter(self):
                super().get_logger().info(
                    f"Drone has entered seeking mode; "
                    f"Looking for new targets for the next {self.DURATION.nanoseconds / 1e9}s"
                )

            def update(self):
                if (
                    not self.seen_target
                    or not super().current_gps_position
                    or super().current_mission
                    == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer
                    if (
                        super().current_mission
                        == Mission.GCP_MARKER_ALIGNING_CUASC.value
                        and not self.seen_target
                    ):
                        super().log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    super().timer().reset()

                elif super().timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = super().State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    super().get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

            def exit(self):
                super().get_logger().info(
                    "Exiting seeking mode; Descending alignment system is now active"
                )

    STATUS_LOG_THROTTLE = Duration(seconds=3)

    LANDING_THRESHOLD_ALTITUDE: float = 0.5
    TAKEOFF_THRESHOLD_ALTITUDE: float = 3.0

    def __init__(self):
        super().__init__("guidance2")

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

        self.update_timer = self.create_timer(0.5, self.update)

        self.current_mode = ArduPilotMode.AUTO
        self.guided_mode_request_in_flight = False
        self.current_gps_position: DronePosition | None = None

        self.last_target_label: str | None = None

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

        self.timer: Timer = Timer(self.get_clock())

        self.machine: StateMachine = StateMachine()
        self.machine.start(self.State.WaitForStartup(self))

        self.startup_delay_ended = False
        self._last_log_time_ns: dict[str, int] = {}

        self.get_logger().info("Guidance2 System has been launched")

    def send_new_mode(self, mode: ArduPilotMode):
        new_mode = Mode()
        new_mode.mode = mode
        self.new_mode_pub.publish(new_mode)

    def _log_throttled(self, level: str, key: str, message: str):
        now_ns = self.get_clock().now().nanoseconds
        last_logged_ns = self._last_log_time_ns.get(key)

        if last_logged_ns is not None:
            elapsed_ns = now_ns - last_logged_ns
            if elapsed_ns < Guidance2.STATUS_LOG_THROTTLE.nanoseconds:
                return

        getattr(self.get_logger(), level)(message)
        self._last_log_time_ns[key] = now_ns

    def mode_callback(self, mode: Mode):
        self.current_mode = ArduPilotMode(mode.mode)

        # A mode update arrived; allow future mode requests if needed.
        self.guided_mode_request_in_flight = False

    def target_position_callback(self, target_position: TargetPosition):
        state = self.machine.get_current_state()

        if state is None:
            return

        # Don't process targets until the drone has taken off and the startup delay has elapsed
        if not self.startup_delay_ended:
            return

        # don't process targets if we're not in seeking or aligned descending mode
        if not isinstance(state, (self.State.Seeking, self.State.AlignedDescending)):
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

            if (
                self.current_mission == Mission.GCP_MARKER_ALIGNING_CUASC.value
                or target_position.object_label.lower() == "bullseye"
            ):
                drone_lat = float(self.current_gps_position.latitude)
                drone_lon = float(self.current_gps_position.longitude)
                target_lat = target_position.latitude
                target_lon = target_position.longitude

                # Simple distance approximation (degrees to meters, ~111km per degree)
                lat_diff_m = (target_lat - drone_lat) * 111000
                lon_diff_m = (target_lon - drone_lon) * 111000
                distance_m = (lat_diff_m**2 + lon_diff_m**2) ** 0.5

                self._log_throttled(
                    "info",
                    "target_alignment",
                    f"Target ({target_position.object_label}): lat={target_lat}, lon={target_lon} | "
                    f"Drone: lat={drone_lat}, lon={drone_lon}, alt={new_position.altitude}m | "
                    f"Distance: {distance_m:.2f}m | Sending alignment command",
                )

            self.new_position_pub.publish(new_position)
        elif self.state == OacState.ALIGNED_DESCENDING:
            new_position = NewDronePosition()
            new_position.latitude = target_position.latitude
            new_position.longitude = target_position.longitude

            new_position.altitude = self.descent_alignment_altitude

            self.new_position_pub.publish(new_position)

    def gps_position_callback(self, gps_position: DronePosition):
        if (
            self.current_mission == Mission.GCP_MARKER_ALIGNING_CUASC.value
            and self.timer.elapsed() > self.State.WaitForStartup.DELAY
        ):
            self.get_logger().debug(
                f"Drone position: lat={gps_position.latitude}, lon={gps_position.longitude}, "
                f"alt={gps_position.altitude}m, yaw={gps_position.yaw} rad"
            )
        self.current_gps_position = gps_position

    def update(self):
        self.machine.update()

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
                ) < float(Guidance2.LANDING_THRESHOLD_ALTITUDE):
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
                ) > float(Guidance2.TAKEOFF_THRESHOLD_ALTITUDE):
                    self.send_new_mode(ArduPilotMode.AUTO)

                    self.state = OacState.RETURNING
                    self.get_logger().info(f"Switching state to {self.state.name}")

            case OacState.RETURNING:
                # Nothing to do for now
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Guidance2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
