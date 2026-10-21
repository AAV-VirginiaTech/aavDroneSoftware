#!/usr/bin/env python3

from __future__ import annotations

from typing import cast

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition, TargetPosition
from rclpy.clock import Duration
from rclpy.node import Node

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
ros2 run aav_software guidance2

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

            def enter(self):
                self.guidance.get_logger().info(f"Entering state: {self.get_name()}")

            def exit(self):
                self.guidance.get_logger().info(f"Exiting state: {self.get_name()}")

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

            def send_new_mode(self, mode: ArduPilotMode):
                self.guidance.send_new_mode(mode)

            def run_payload_drop_sequence(
                self,
                openandclose: bool,
                pwm: float,
                aux_port: int,
            ) -> None:
                run_payload_drop_sequence(self.guidance, openandclose, pwm, aux_port)

            def last_target_label(self) -> str | None:
                return self.guidance.last_target_label

            def new_position_pub(self):
                return self.guidance.new_position_pub

            def substructure_action_duration(self):
                return self.guidance.substructure_action_duration

            def seen_target(self):
                return self.guidance.seen_target

            def set_seen_target(self, cond):
                self.guidance.seen_target = cond

        class WaitForStartup(Base):
            """
            Waiting for the drone to take off and for the startup delay to elapse
            """

            DELAY = Duration(seconds=45)

            def __init__(self, guidance: Guidance2):
                super().__init__("wait_for_startup", guidance)
                self.wait_started = False

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Waiting for takeoff above {Guidance2.TAKEOFF_THRESHOLD_ALTITUDE}m; "
                    f"Startup delay: {self.DELAY.nanoseconds / 1e9}s"
                )

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Startup delay complete; Target tracking enabled"
                )

            def update(self):
                super().update()
                gps_pos = self.current_gps_position()
                if not gps_pos:
                    return

                current_altitude = float(gps_pos.altitude)
                threshold_altitude = float(Guidance2.TAKEOFF_THRESHOLD_ALTITUDE)

                # Start the startup delay timer once drone takes off
                if not self.wait_started and current_altitude > threshold_altitude:
                    self.timer().reset()
                    self.get_logger().info(
                        f"Drone has taken off at altitude {gps_pos.altitude}m; "
                        f"starting {self.DELAY.nanoseconds / 1e9}s startup delay"
                    )
                    self.wait_started = True
                    return

                # check if we've waited longer than the startup delay
                waited = self.timer().elapsed()
                if self.wait_started and waited > self.DELAY:
                    # initiate seeking mode
                    self.get_guidance().startup_delay_ended = True
                    self.get_logger().info(
                        "Startup delay has ended; GCP marker tracking is now active"
                    )
                    self.mark_finished(Guidance2.State.Seeking(self.guidance))
                    return

                # debug waiting state
                current_mission = self.current_mission()
                goal_mission = Mission.GCP_MARKER_ALIGNING_CUASC.value
                if current_mission == goal_mission:
                    self.log_throttled(
                        "info",
                        "gcp_startup_delay_progress",
                        f"GCP Mission: Startup delay in progress ({waited:.1f}s / "
                        f"{self.DELAY.nanoseconds / 1e9}s)",
                    )

        class Seeking(Base):
            """
            Monitoring for new targets; sending their positions while maintaining altitude
            """

            DURATION = Duration(seconds=30)

            def __init__(self, guidance: Guidance2):
                super().__init__("seeking", guidance)
                self.guidance: Guidance2 = guidance

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Monitoring for targets while maintaining altitude; "
                    f"Tracking duration: {self.DURATION.nanoseconds / 1e9}s"
                )

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Seeking phase complete; Beginning aligned descent"
                )

            def update(self):
                super().update()
                if (
                    not self.seen_target()
                    or not self.current_gps_position()
                    or self.current_mission() == Mission.GCP_MARKER_ALIGNING_CUASC.value
                ):
                    # if we have seen no targets, or the gps is not connected, reset the timer

                    current_mission = self.current_mission()
                    goal_mission = Mission.GCP_MARKER_ALIGNING_CUASC.value

                    if current_mission == goal_mission and not self.seen_target():
                        self.log_throttled(
                            "debug",
                            "gcp_waiting_for_first_target",
                            "GCP Mission: Waiting for first target detection",
                        )
                    self.timer().reset()

                elif self.timer().elapsed() > self.DURATION:
                    # if the timer has expired (we saw our first target 60 seconds ago), start descending
                    next_state = Guidance2.State.AlignedDescending(self.guidance)
                    self.mark_finished(next_state)
                    self.get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

        class AlignedDescending(Base):
            """
            Sending target positions while descending to alignment altitude; holds at alignment altitude for 8s
            """

            DURATION = Duration(seconds=20)

            def __init__(self, guidance: Guidance2):
                super().__init__("aligned_descending", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Descending to alignment altitude of "
                    f"{self.descent_alignment_altitude()}m; "
                    f"Holding within 0.25m for {self.DURATION.nanoseconds / 1e9}s"
                )

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Alignment hold complete; Beginning final descent"
                )

            def update(self):
                super().update()
                gps_pos = self.current_gps_position()
                if not gps_pos:
                    self.timer().reset()
                    return

                # within .25m of drop altitude, check timer
                altitude_dif = abs(gps_pos.altitude - self.descent_alignment_altitude())
                if altitude_dif < 0.25:
                    # allow 8 seconds for final alignment
                    if self.timer().elapsed() < self.DURATION:
                        pass
                    # if the timer has expired, move on to final descent
                    else:
                        next_state = Guidance2.State.FinalDescending(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )
                # not within range of drop altitude, reset the timer
                else:
                    self.timer().reset()

        class FinalDescending(Base):
            """
            Descending to drop altitude
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("final_descending", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Descending to final drop altitude of "
                    f"{self.hardcoded_drop_altitude()}m"
                )

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Final descent complete; Proceeding with mission action"
                )

            def update(self):
                super().update()
                gps_pos = self.current_gps_position()
                if not gps_pos:
                    return

                # if within .25m of drop alititude
                if abs(gps_pos.altitude - self.hardcoded_drop_altitude()) < 0.25:
                    # if at descent altitude, either start landing or run payload drop
                    if self.current_mission() == Mission.PACKAGE_DELIVERY_CUASC.value:
                        next_state = Guidance2.State.Landing(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )
                    elif self.current_mission() == Mission.PAYLOAD_DROP_CUASC.value:
                        self.get_logger().info("Dropping payload")
                        try:
                            self.run_payload_drop_sequence(True, 2000, 8)
                        except Exception as exc:
                            self.get_logger().error(
                                f"Payload drop sequence failed: {exc}"
                            )
                        self.timer().reset()

                        next_state = Guidance2.State.DroppingPayload(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )
                    elif self.current_mission() == Mission.PAYLOAD_DELIVERY_SUAS.value:
                        self.timer().reset()
                        if self.last_target_label() == "manikin":
                            self.get_logger().info("Dropping water bottle")

                            try:
                                self.run_payload_drop_sequence(True, 2000, 7)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )
                        else:
                            self.get_logger().info("Dropping beacon")
                            try:
                                self.run_payload_drop_sequence(True, 2000, 6)
                            except Exception as exc:
                                self.get_logger().error(
                                    f"Payload drop sequence failed: {exc}"
                                )

                        next_state = Guidance2.State.DroppingPayload(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )

                else:
                    new_position = NewDronePosition()

                    gps_pos = self.current_gps_position()
                    if gps_pos:
                        new_position.longitude = gps_pos.longitude
                        new_position.latitude = gps_pos.latitude
                        new_position.altitude = self.hardcoded_drop_altitude()
                        self.new_position_pub().publish(new_position)

        class DroppingPayload(Base):
            """
            Waiting for payload drop to finish
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("dropping_payload", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Payload release commanded; "
                    f"Waiting {self.substructure_action_duration().nanoseconds / 1e9}s "
                    f"before continuing mission"
                )

            def update(self):
                super().update()
                if self.timer().elapsed() > self.substructure_action_duration():
                    if self.current_mission() == Mission.PAYLOAD_DELIVERY_SUAS.value:
                        self.set_seen_target(False)
                        next_state = Guidance2.State.Seeking(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )
                    else:
                        next_state = Guidance2.State.Returning(self.guidance)
                        self.mark_finished(next_state)
                        self.get_logger().info(
                            f"Switching state to {next_state.get_name()}"
                        )

        class Landing(Base):
            """
            In landing mode; waiting to reach ground
            """

            THRESHOLD_ALTITUDE: float = 0.5

            def __init__(self, guidance: Guidance2):
                super().__init__("landing", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Landing initiated; Waiting for altitude below "
                    f"{self.THRESHOLD_ALTITUDE}m"
                )
                self.send_new_mode(ArduPilotMode.LAND)

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Landing altitude threshold reached; Package release commanded"
                )

            def update(self):
                super().update()
                gps_pos = self.current_gps_position()
                if not gps_pos:
                    return

                # if within .25m of drop alititude
                if gps_pos.altitude < self.THRESHOLD_ALTITUDE:
                    self.get_logger().info("Dropping CUASC package")
                    try:
                        self.run_payload_drop_sequence(False, 1500, 8)
                    except Exception as exc:
                        self.get_logger().error(f"Payload drop sequence failed: {exc}")

                    self.timer().reset()
                    next_state = Guidance2.State.DroppingPackage(self.guidance)
                    self.mark_finished(next_state)
                    self.get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

        class DroppingPackage(Base):
            """
            Waiting for package drop to finish
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("dropping_package", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Package release commanded; "
                    f"Waiting {self.substructure_action_duration().nanoseconds / 1e9}s "
                    f"before takeoff"
                )

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Package release wait complete; Takeoff requested"
                )

            def update(self):
                super().update()
                if self.timer().elapsed() > self.substructure_action_duration():
                    next_state = Guidance2.State.TakingOff(self.guidance)
                    self.mark_finished(next_state)
                    self.get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

        class TakingOff(Base):
            """
            In takeoff mode; waiting to reach threshold altitude
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("taking_off", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    f"Takeoff initiated; Waiting to exceed "
                    f"{Guidance2.TAKEOFF_THRESHOLD_ALTITUDE}m altitude"
                )

                self.send_new_mode(ArduPilotMode.TAKEOFF)

            def exit(self):
                super().exit()
                self.get_logger().info(
                    "Takeoff altitude threshold reached; AUTO mode requested"
                )

            def update(self):
                super().update()
                gps_pos = self.current_gps_position()
                if not gps_pos:
                    return

                # if within .25m of drop alititude
                if gps_pos.altitude > Guidance2.TAKEOFF_THRESHOLD_ALTITUDE:
                    next_state = Guidance2.State.Returning(self.guidance)
                    self.mark_finished(next_state)
                    self.get_logger().info(
                        f"Switching state to {next_state.get_name()}"
                    )

        class Returning(Base):
            """
            Back in AUTO mode; done
            """

            def __init__(self, guidance: Guidance2):
                super().__init__("returning", guidance)

            def enter(self):
                super().enter()
                self.get_logger().info(
                    "Guidance sequence complete; Returning control to AUTO mode"
                )

                self.send_new_mode(ArduPilotMode.AUTO)

            def update(self):
                super().update()

    STATUS_LOG_THROTTLE = Duration(seconds=3)
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
        self.debug_timer = self.create_timer(5, self.debug_timer_callback)

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
        self.seen_target = False
        self.timer: Timer = Timer(self.get_clock())

        self.machine: StateMachine = StateMachine(self.get_clock())

        self.startup_delay_ended = False
        self._last_log_time_ns: dict[str, int] = {}

        self.get_logger().info("Guidance2 System has been launched")
        self.machine.start(self.State.WaitForStartup(self))

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

        if isinstance(self.machine.current_state, self.State.Seeking):
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

        if isinstance(self.machine.current_state, self.State.AlignedDescending):
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

    def debug_timer_callback(self):
        state = self.machine.get_current_state()
        name = state.get_name() if state else "nothing"
        self.get_logger().info(
            f"Working on {name}... for {self.machine.get_state_time().nanoseconds / 1e9}s"
        )

    def update(self):
        self.machine.update()


def main(args=None):
    rclpy.init(args=args)
    node = Guidance2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
