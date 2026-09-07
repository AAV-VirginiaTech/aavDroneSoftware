#!/usr/bin/env python3
import math
import time
from typing import cast

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from .topic_converter_for_simulation import ArduPilotMode

# =========================
#  MODE MAPPING
# =========================

MODE_TO_STRING = {
    ArduPilotMode.STABILIZE.value: "STABILIZE",
    ArduPilotMode.ACRO.value: "ACRO",
    ArduPilotMode.ALT_HOLD.value: "ALT_HOLD",
    ArduPilotMode.GUIDED.value: "GUIDED",
    ArduPilotMode.AUTO.value: "AUTO",
    ArduPilotMode.LOITER.value: "LOITER",
    ArduPilotMode.RTL.value: "RTL",
    ArduPilotMode.CIRCLE.value: "CIRCLE",
    8: "POSITION",
    ArduPilotMode.LAND.value: "LAND",
    10: "OF_LOITER",
    ArduPilotMode.DRIFT.value: "DRIFT",
    ArduPilotMode.SPORT.value: "SPORT",
    ArduPilotMode.FLIP.value: "FLIP",
    ArduPilotMode.AUTOTUNE.value: "AUTOTUNE",
    ArduPilotMode.POSHOLD.value: "POSHOLD",
    ArduPilotMode.BRAKE.value: "BRAKE",
    ArduPilotMode.THROW.value: "THROW",
    ArduPilotMode.AVOID_ADSB.value: "AVOID_ADSB",
    ArduPilotMode.GUIDED_NOGPS.value: "GUIDED_NOGPS",
    ArduPilotMode.SMART_RTL.value: "SMART_RTL",
    ArduPilotMode.FLOWHOLD.value: "FLOWHOLD",
    ArduPilotMode.FOLLOW.value: "FOLLOW",
    ArduPilotMode.ZIGZAG.value: "ZIGZAG",
    ArduPilotMode.SYSTEMID.value: "SYSTEMID",
    ArduPilotMode.AUTOROTATE.value: "AUTOROTATE",
    ArduPilotMode.AUTO_RTL.value: "AUTO_RTL",
    ArduPilotMode.TURTLE.value: "TURTLE",
}

STRING_TO_MODE = {
    "STABILIZE": ArduPilotMode.STABILIZE.value,
    "ACRO": ArduPilotMode.ACRO.value,
    "ALT_HOLD": ArduPilotMode.ALT_HOLD.value,
    "GUIDED": ArduPilotMode.GUIDED.value,
    "AUTO": ArduPilotMode.AUTO.value,
    "LOITER": ArduPilotMode.LOITER.value,
    "RTL": ArduPilotMode.RTL.value,
    "CIRCLE": ArduPilotMode.CIRCLE.value,
    "LAND": ArduPilotMode.LAND.value,
    "DRIFT": ArduPilotMode.DRIFT.value,
    "APPROACH": ArduPilotMode.DRIFT.value,
    "SPORT": ArduPilotMode.SPORT.value,
    "FLIP": ArduPilotMode.FLIP.value,
    "AUTOTUNE": ArduPilotMode.AUTOTUNE.value,
    "POSHOLD": ArduPilotMode.POSHOLD.value,
    "BRAKE": ArduPilotMode.BRAKE.value,
    "THROW": ArduPilotMode.THROW.value,
    "AVOID_ADSB": ArduPilotMode.AVOID_ADSB.value,
    "GUIDED_NOGPS": ArduPilotMode.GUIDED_NOGPS.value,
    "SMART_RTL": ArduPilotMode.SMART_RTL.value,
    "FLOWHOLD": ArduPilotMode.FLOWHOLD.value,
    "FOLLOW": ArduPilotMode.FOLLOW.value,
    "ZIGZAG": ArduPilotMode.ZIGZAG.value,
    "SYSTEMID": ArduPilotMode.SYSTEMID.value,
    "AUTOROTATE": ArduPilotMode.AUTOROTATE.value,
    "AUTO_RTL": ArduPilotMode.AUTO_RTL.value,
    "TURTLE": ArduPilotMode.TURTLE.value,
    # MAVROS exposes these copter mode names for numeric values not in ArduPilot's standard list.
    "POSITION": 8,
    "OF_LOITER": 10,
}


class TopicConverter(Node):
    FLOAT32_MAX = 3.402823466e38

    # Retain position and ENU yaw; MAVROS converts yaw to the FCU convention.
    POSITION_AND_YAW_MASK = (
        GlobalPositionTarget.IGNORE_VX
        | GlobalPositionTarget.IGNORE_VY
        | GlobalPositionTarget.IGNORE_VZ
        | GlobalPositionTarget.IGNORE_AFX
        | GlobalPositionTarget.IGNORE_AFY
        | GlobalPositionTarget.IGNORE_AFZ
        | GlobalPositionTarget.IGNORE_YAW_RATE
    )

    def __init__(self):
        super().__init__("topic_converter_for_drone")
        self.get_logger().info("MAVROS Topic Converter (Hardcoded Modes) Started")

        self.connected = False
        self.current_latitude: float | None = None
        self.current_longitude: float | None = None
        self.current_relative_altitude: float | None = None
        self.current_yaw: float | None = None
        self.current_mode = None
        self.gps_received_at: float | None = None
        self.relative_altitude_received_at: float | None = None
        self.imu_received_at: float | None = None
        self.wind_velocity_enu: tuple[float, float] | None = None
        self.wind_received_at: float | None = None

        self.telemetry_timeout_sec = cast(
            float, self.declare_parameter("telemetry_timeout_sec", 2.0).value
        )
        if (
            not math.isfinite(self.telemetry_timeout_sec)
            or self.telemetry_timeout_sec <= 0.0
        ):
            raise ValueError("telemetry_timeout_sec must be finite and positive")

        self.latest_setpoint: GlobalPositionTarget | None = None
        self.last_setpoint_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =========================
        # SUBSCRIBERS (MAVROS)
        # =========================

        self.create_subscription(State, "/mavros/state", self.state_callback, 10)

        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, sensor_qos
        )

        self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self.relative_altitude_callback,
            sensor_qos,
        )

        # The local-position plugin can publish a default orientation without IMU
        # data. Subscribe to actual attitude updates so readiness is observable.
        self.create_subscription(Imu, "/mavros/imu/data", self.imu_callback, sensor_qos)

        # ArduPilot's onboard EKF estimate; no external wind sensor required.
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/mavros/wind_estimation",
            self.wind_callback,
            sensor_qos,
        )

        # =========================
        # SUBSCRIBERS (AAV)
        # =========================

        self.create_subscription(Mode, "/AAV/set_mode", self.set_mode_callback, 10)

        self.create_subscription(
            NewDronePosition, "/AAV/send_new_position", self.new_position_callback, 10
        )

        # =========================
        # PUBLISHERS
        # =========================

        self.mode_pub = self.create_publisher(Mode, "/AAV/current_mode", 10)

        self.gps_pub = self.create_publisher(
            DronePosition, "/AAV/current_gps_position", 10
        )

        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 10
        )

        # Assemble the latest independently received telemetry at a bounded rate.
        self.create_timer(0.1, self.publish_drone_position)
        # Refresh accepted goals at 5 Hz while telemetry and the goal are fresh.
        self.create_timer(0.2, self.publish_setpoint)
        self.create_timer(5.0, self.log_wind_direction)

    # =========================
    # UTIL FUNCTIONS
    # =========================

    def quaternion_to_yaw(self, q):
        components = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in components):
            raise ValueError("IMU orientation contains nonfinite values")
        norm = math.hypot(*components)
        if abs(norm - 1.0) > 0.01:
            raise ValueError("IMU orientation is not a unit quaternion")
        x, y, z, w = (value / norm for value in components)
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    def _telemetry_ready(self):
        if not self.connected or any(
            value is None
            for value in (
                self.current_latitude,
                self.current_longitude,
                self.current_relative_altitude,
                self.current_yaw,
            )
        ):
            return False
        now = time.monotonic()
        return all(
            received_at is not None
            and 0.0 <= now - received_at <= self.telemetry_timeout_sec
            for received_at in (
                self.gps_received_at,
                self.relative_altitude_received_at,
                self.imu_received_at,
            )
        )

    def _clear_telemetry(self):
        self.current_latitude = None
        self.current_longitude = None
        self.current_relative_altitude = None
        self.current_yaw = None
        self.gps_received_at = None
        self.relative_altitude_received_at = None
        self.imu_received_at = None
        self.wind_velocity_enu = None
        self.wind_received_at = None
        self.latest_setpoint = None

    def _valid_coordinates(self, latitude, longitude):
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    def _valid_altitude(self, altitude):
        # Input altitude is float64, but both output messages use float32.
        return math.isfinite(altitude) and abs(altitude) <= self.FLOAT32_MAX

    def call_service(self, client, req, name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} not available")
            return None

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    # =========================
    # MAVROS CALLBACKS
    # =========================

    def wind_callback(self, msg: TwistWithCovarianceStamped):
        velocity = msg.twist.twist.linear
        # MAVROS publishes wind velocity TO the destination in earth-fixed ENU.
        # ArduPilot sets covariance[0] = -1 for unknown covariance, not bad wind.
        if not self.connected or not all(
            math.isfinite(value) for value in (velocity.x, velocity.y)
        ):
            self.wind_velocity_enu = None
            self.wind_received_at = None
            return
        self.wind_velocity_enu = (velocity.x, velocity.y)
        self.wind_received_at = time.monotonic()

    def log_wind_direction(self):
        if (
            not self.connected
            or self.wind_velocity_enu is None
            or self.wind_received_at is None
            or not 0.0
            <= time.monotonic() - self.wind_received_at
            <= self.telemetry_timeout_sec
        ):
            self.get_logger().info(
                "Wind direction unavailable: waiting for a fresh onboard EKF "
                "estimate on /mavros/wind_estimation. Check EKF3 drag estimation "
                "and the MAVLink WIND stream."
            )
            return

        east, north = self.wind_velocity_enu
        speed = math.hypot(east, north)
        if speed < 0.1:
            self.get_logger().info(
                "Estimated wind direction undefined: horizontal wind below 0.1 m/s"
            )
            return

        # Meteorological FROM bearing: clockwise from North (N=0, E=90).
        direction = math.degrees(math.atan2(-east, -north)) % 360.0
        compass = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        cardinal = compass[int((direction + 22.5) / 45.0) % len(compass)]
        self.get_logger().info(
            f"Estimated wind FROM {direction:.1f} deg ({cardinal}), "
            f"horizontal speed {speed:.2f} m/s (onboard EKF)"
        )

    def state_callback(self, msg: State):
        if not msg.connected or msg.connected != self.connected:
            # Require new samples after reconnect, not cached pre-disconnect data.
            self._clear_telemetry()
        self.connected = msg.connected
        self.current_mode = msg.mode

        if msg.mode in STRING_TO_MODE:
            mode_msg = Mode()
            mode_msg.mode = STRING_TO_MODE[msg.mode]
            self.mode_pub.publish(mode_msg)

    def imu_callback(self, msg: Imu):
        try:
            if msg.orientation_covariance[0] < 0:
                raise ValueError("IMU reports orientation unavailable")
            yaw = self.quaternion_to_yaw(msg.orientation)
        except ValueError as exc:
            self.current_yaw = None
            self.imu_received_at = None
            self.latest_setpoint = None
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return

        first_orientation = self.current_yaw is None
        self.current_yaw = yaw
        self.imu_received_at = time.monotonic()
        if first_orientation:
            self.get_logger().info(
                f"Received MAVROS IMU attitude: ENU yaw={yaw:.3f} rad"
            )

    def relative_altitude_callback(self, msg: Float64):
        if not self._valid_altitude(msg.data):
            self.current_relative_altitude = None
            self.relative_altitude_received_at = None
            self.latest_setpoint = None
            self.get_logger().warning(
                "Ignoring invalid FCU relative altitude", throttle_duration_sec=5.0
            )
            return
        # Already metres above FCU home; never derive home from the first GPS fix.
        self.current_relative_altitude = msg.data
        self.relative_altitude_received_at = time.monotonic()

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX or not self._valid_coordinates(
            msg.latitude, msg.longitude
        ):
            self.current_latitude = None
            self.current_longitude = None
            self.gps_received_at = None
            self.latest_setpoint = None
            self.get_logger().warning(
                "Ignoring invalid MAVROS global position", throttle_duration_sec=5.0
            )
            return

        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        self.gps_received_at = time.monotonic()

    def publish_drone_position(self):
        if not self._telemetry_ready():
            self.get_logger().warning(
                "Waiting for FCU connection and fresh GPS, relative altitude, and IMU attitude",
                throttle_duration_sec=5.0,
            )
            return
        assert self.current_latitude is not None
        assert self.current_longitude is not None
        assert self.current_relative_altitude is not None
        assert self.current_yaw is not None
        gps_msg = DronePosition()
        gps_msg.latitude = self.current_latitude
        gps_msg.longitude = self.current_longitude
        gps_msg.altitude = self.current_relative_altitude
        gps_msg.yaw = self.current_yaw

        self.gps_pub.publish(gps_msg)

    # =========================
    # AAV CALLBACKS
    # =========================

    def set_mode_callback(self, msg: Mode):

        if self.current_mode == "POSHOLD":
            self.get_logger().warn("Cannot switch out of POSHOLD")
            return

        if msg.mode == ArduPilotMode.TAKEOFF.value:
            self.takeoff(30.0)
            return

        if msg.mode not in MODE_TO_STRING:
            self.get_logger().error("Unknown mode")
            return

        mode_string = MODE_TO_STRING[msg.mode]

        client = self.create_client(SetMode, "/mavros/set_mode")

        req = SetMode.Request()
        req.custom_mode = mode_string

        self.call_service(client, req, "set_mode")

    def new_position_callback(self, msg: NewDronePosition):
        if not self._telemetry_ready():
            self.latest_setpoint = None
            self.get_logger().warning(
                "Rejecting position goal: FCU telemetry unavailable or stale",
                throttle_duration_sec=5.0,
            )
            return
        if not self._valid_coordinates(
            msg.latitude, msg.longitude
        ) or not self._valid_altitude(msg.altitude):
            self.latest_setpoint = None
            self.get_logger().warning(
                "Rejecting nonfinite or out-of-range position goal"
            )
            return
        assert self.current_yaw is not None
        target = GlobalPositionTarget()
        target.header.frame_id = "map"
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = self.POSITION_AND_YAW_MASK
        target.latitude = msg.latitude
        target.longitude = msg.longitude
        target.altitude = msg.altitude
        # Hold the measured heading at acceptance; never refresh it in the timer.
        # This is ROS ENU radians. The MAVROS raw plugin performs ENU -> NED.
        target.yaw = self.current_yaw

        self.latest_setpoint = target
        self.last_setpoint_time = time.monotonic()

    def publish_setpoint(self):
        if self.latest_setpoint is None:
            return

        if (
            not self._telemetry_ready()
            or time.monotonic() - self.last_setpoint_time > 5.0
        ):
            # Do not resume this goal after telemetry recovers. Stopping sends
            # does not cancel a position target already accepted by the FCU.
            self.latest_setpoint = None
            return

        self.latest_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.latest_setpoint)

    # =========================
    # SERVICES
    # =========================

    def arm(self):
        client = self.create_client(CommandBool, "/mavros/cmd/arming")
        req = CommandBool.Request()
        req.value = True
        return self.call_service(client, req, "arming")

    def takeoff(self, altitude):
        if not self._telemetry_ready():
            self.get_logger().warning("Cannot take off without fresh FCU telemetry")
            return
        assert self.current_latitude is not None
        assert self.current_longitude is not None
        assert self.current_yaw is not None
        latitude = self.current_latitude
        longitude = self.current_longitude
        yaw = self.current_yaw

        # Set GUIDED
        self.set_mode_callback(Mode(mode=ArduPilotMode.GUIDED.value))

        # Arm
        self.arm()

        # Takeoff
        client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = latitude
        req.longitude = longitude
        # CommandTOL yaw is a compass heading in degrees (Copter ignores it).
        req.yaw = (90.0 - math.degrees(yaw)) % 360.0

        self.call_service(client, req, "takeoff")


def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
