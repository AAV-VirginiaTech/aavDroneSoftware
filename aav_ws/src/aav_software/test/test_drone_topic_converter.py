"""Regression coverage for the hardware converter's altitude and attitude contract."""

import math
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import Mock

import pytest
from aav_msgs.msg import NewDronePosition
from aav_software import topic_converter_for_drone as converter
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64


@pytest.fixture
def node(monkeypatch):
    """Exercise production callbacks without starting a ROS executor or services."""
    instance = cast(Any, converter.TopicConverter.__new__(converter.TopicConverter))
    instance.connected = False
    instance.current_mode = None
    instance.current_latitude = None
    instance.current_longitude = None
    instance.current_relative_altitude = None
    instance.current_yaw = None
    instance.gps_received_at = None
    instance.relative_altitude_received_at = None
    instance.imu_received_at = None
    instance.wind_velocity_enu = None
    instance.wind_received_at = None
    instance.telemetry_timeout_sec = 2.0
    instance.latest_setpoint = None
    instance.last_setpoint_time = 0.0
    instance.mode_pub = Mock()
    instance.gps_pub = Mock()
    instance.setpoint_pub = Mock()
    instance.get_logger = Mock(return_value=Mock())
    instance.get_clock = Mock(
        return_value=SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))
        )
    )
    monkeypatch.setattr(converter.time, "monotonic", lambda: 100.0)
    return instance


def gps_fix(altitude=500.0):
    msg = NavSatFix()
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.latitude = 37.2296
    msg.longitude = -80.4139
    msg.altitude = altitude
    return msg


def imu_attitude(yaw=0.0, roll=0.0, pitch=0.0):
    """Build a full body-to-ENU quaternion, including aircraft roll and pitch."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    msg = Imu()
    msg.orientation = Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )
    return msg


def connect_with_telemetry(node, altitude=25.0, yaw=0.0):
    node.state_callback(State(connected=True, mode="GUIDED"))
    node.gps_callback(gps_fix())
    node.relative_altitude_callback(Float64(data=altitude))
    node.imu_callback(imu_attitude(yaw=yaw))


def position_goal(altitude=20.0):
    return NewDronePosition(latitude=37.2297, longitude=-80.4138, altitude=altitude)


def test_init_wires_relative_altitude_and_real_imu_with_sensor_qos(monkeypatch):
    monkeypatch.setattr(converter.Node, "__init__", lambda self, name: None)
    monkeypatch.setattr(converter.Node, "get_logger", Mock(return_value=Mock()))
    monkeypatch.setattr(
        converter.Node,
        "declare_parameter",
        Mock(return_value=SimpleNamespace(value=2.0)),
    )
    subscriptions, publishers, timers = Mock(), Mock(), Mock()
    monkeypatch.setattr(converter.Node, "create_subscription", subscriptions)
    monkeypatch.setattr(converter.Node, "create_publisher", publishers)
    monkeypatch.setattr(converter.Node, "create_timer", timers)

    instance = converter.TopicConverter()

    by_topic = {call.args[1]: call.args for call in subscriptions.call_args_list}
    for topic, message_type, callback in (
        ("/mavros/global_position/global", NavSatFix, instance.gps_callback),
        (
            "/mavros/global_position/rel_alt",
            Float64,
            instance.relative_altitude_callback,
        ),
        ("/mavros/imu/data", Imu, instance.imu_callback),
        (
            "/mavros/wind_estimation",
            TwistWithCovarianceStamped,
            instance.wind_callback,
        ),
    ):
        assert by_topic[topic][0] is message_type
        assert by_topic[topic][2] == callback
        assert by_topic[topic][3].reliability == ReliabilityPolicy.BEST_EFFORT
    assert "/mavros/local_position/pose" not in by_topic
    publishers.assert_any_call(GlobalPositionTarget, "/mavros/setpoint_raw/global", 10)
    timers.assert_any_call(0.1, instance.publish_drone_position)
    timers.assert_any_call(0.2, instance.publish_setpoint)
    timers.assert_any_call(5.0, instance.log_wind_direction)
    assert instance.current_relative_altitude is None
    assert instance.current_yaw is None
    assert not instance._telemetry_ready()


def wind_estimate(east, north):
    msg = TwistWithCovarianceStamped()
    msg.twist.twist.linear.x = float(east)
    msg.twist.twist.linear.y = float(north)
    # This is the normal ArduPilot WIND covariance convention in MAVROS.
    msg.twist.covariance[0] = -1.0
    return msg


@pytest.mark.parametrize(
    "east,north,bearing,cardinal",
    [
        (0, -2, 0, "N"),
        (-2, -2, 45, "NE"),
        (-2, 0, 90, "E"),
        (-2, 2, 135, "SE"),
        (0, 2, 180, "S"),
        (2, 2, 225, "SW"),
        (2, 0, 270, "W"),
        (2, -2, 315, "NW"),
    ],
)
def test_wind_logs_from_bearing_in_compass_degrees(
    node, east, north, bearing, cardinal
):
    node.state_callback(State(connected=True))
    node.wind_callback(wind_estimate(east, north))
    # Receipt alone must not log; the five-second timer controls output.
    node.get_logger().info.assert_not_called()

    node.log_wind_direction()

    node.get_logger().info.assert_called_once_with(
        f"Estimated wind FROM {bearing:.1f} deg ({cardinal}), "
        f"horizontal speed {math.hypot(east, north):.2f} m/s (onboard EKF)"
    )


@pytest.mark.parametrize("east,north", [(0, 0), (0.03, 0.04)])
def test_near_zero_wind_does_not_invent_a_direction(node, east, north):
    node.state_callback(State(connected=True))
    node.wind_callback(wind_estimate(east, north))
    node.log_wind_direction()
    assert "direction undefined" in node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize("east,north", [(math.nan, 2), (1, math.inf), (-math.inf, 0)])
def test_invalid_wind_replaces_previous_estimate(node, east, north):
    node.state_callback(State(connected=True))
    node.wind_callback(wind_estimate(0, -2))
    node.wind_callback(wind_estimate(east, north))
    node.log_wind_direction()
    assert node.wind_velocity_enu is None
    assert "direction unavailable" in node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize("received_at", [None, 97.9, 101.0])
def test_missing_or_stale_wind_is_not_logged_as_current(node, received_at):
    node.state_callback(State(connected=True))
    node.wind_callback(wind_estimate(0, -2))
    node.wind_received_at = received_at
    node.log_wind_direction()
    assert "direction unavailable" in node.get_logger().info.call_args.args[0]


def test_wind_requires_new_sample_after_disconnect(node):
    node.state_callback(State(connected=True))
    node.wind_callback(wind_estimate(0, -2))
    node.state_callback(State(connected=False))
    node.wind_callback(wind_estimate(2, 0))
    assert node.wind_velocity_enu is None
    node.state_callback(State(connected=True))
    node.log_wind_direction()
    assert "direction unavailable" in node.get_logger().info.call_args.args[0]


def test_wind_is_optional_for_position_telemetry(node):
    connect_with_telemetry(node)
    node.log_wind_direction()
    node.publish_drone_position()
    assert node._telemetry_ready()
    node.gps_pub.publish.assert_called_once()


def test_airborne_startup_uses_fcu_relative_altitude_without_zeroing(node):
    connect_with_telemetry(node, altitude=25.0)

    node.publish_drone_position()

    node.gps_pub.publish.assert_called_once()
    position = node.gps_pub.publish.call_args.args[0]
    assert position.latitude == pytest.approx(37.2296)
    assert position.longitude == pytest.approx(-80.4139)
    assert position.altitude == pytest.approx(25.0)
    assert position.yaw == pytest.approx(0.0)


@pytest.mark.parametrize("gps_altitude", [-30.0, 500.0, 4000.0, math.nan])
def test_gps_ellipsoid_altitude_never_changes_relative_altitude_or_goal(
    node, gps_altitude
):
    connect_with_telemetry(node, altitude=25.0)
    node.gps_callback(gps_fix(altitude=gps_altitude))

    node.publish_drone_position()
    node.new_position_callback(position_goal(altitude=20.0))
    node.publish_setpoint()

    assert node.gps_pub.publish.call_args.args[0].altitude == pytest.approx(25.0)
    assert node.setpoint_pub.publish.call_args.args[0].altitude == pytest.approx(20.0)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2])
@pytest.mark.parametrize("roll,pitch", [(0.0, 0.0), (0.2, -0.3)])
def test_imu_cardinal_yaw_survives_roll_and_pitch(node, yaw, roll, pitch):
    connect_with_telemetry(node)
    node.imu_callback(imu_attitude(yaw=yaw, roll=roll, pitch=pitch))

    node.publish_drone_position()
    node.new_position_callback(position_goal())

    actual_yaw = node.gps_pub.publish.call_args.args[0].yaw
    assert math.sin(actual_yaw - yaw) == pytest.approx(0.0, abs=1e-7)
    assert math.cos(actual_yaw - yaw) == pytest.approx(1.0, abs=1e-7)
    assert node.latest_setpoint.yaw == pytest.approx(yaw, abs=1e-6)


def test_raw_global_goal_enables_position_and_enu_yaw_with_home_relative_altitude(node):
    connect_with_telemetry(node, yaw=math.pi / 2)
    goal = position_goal(altitude=20.0)

    node.new_position_callback(goal)
    node.publish_setpoint()

    target = node.setpoint_pub.publish.call_args.args[0]
    assert isinstance(target, GlobalPositionTarget)
    assert target.coordinate_frame == GlobalPositionTarget.FRAME_GLOBAL_REL_ALT == 6
    assert target.type_mask == 2552
    assert (
        target.type_mask
        & (
            GlobalPositionTarget.IGNORE_LATITUDE
            | GlobalPositionTarget.IGNORE_LONGITUDE
            | GlobalPositionTarget.IGNORE_ALTITUDE
            | GlobalPositionTarget.IGNORE_YAW
        )
        == 0
    )
    assert target.latitude == goal.latitude
    assert target.longitude == goal.longitude
    assert target.altitude == pytest.approx(20.0)
    # MAVROS owns ENU-to-NED conversion; this layer must not pre-convert North to zero.
    assert target.yaw == pytest.approx(math.pi / 2)
    assert target.header.stamp.sec == 123


@pytest.mark.parametrize(
    "timestamp_field",
    ["gps_received_at", "relative_altitude_received_at", "imu_received_at"],
)
@pytest.mark.parametrize("timestamp", [None, 97.9])
def test_missing_or_stale_sensor_blocks_position_and_new_goals(
    node, timestamp_field, timestamp
):
    connect_with_telemetry(node)
    setattr(node, timestamp_field, timestamp)

    node.publish_drone_position()
    node.new_position_callback(position_goal())

    assert not node._telemetry_ready()
    node.gps_pub.publish.assert_not_called()
    assert node.latest_setpoint is None


def test_zero_yaw_requires_an_actual_imu_sample(node):
    node.state_callback(State(connected=True, mode="GUIDED"))
    node.gps_callback(gps_fix())
    node.relative_altitude_callback(Float64(data=25.0))
    node.publish_drone_position()
    assert node.current_yaw is None
    node.gps_pub.publish.assert_not_called()

    node.imu_callback(imu_attitude(yaw=0.0))
    node.publish_drone_position()
    assert node._telemetry_ready()
    assert node.gps_pub.publish.call_args.args[0].yaw == pytest.approx(0.0)


@pytest.mark.parametrize(
    "quaternion",
    [
        Quaternion(),
        Quaternion(w=2.0),
        Quaternion(x=math.nan, w=1.0),
        Quaternion(z=math.inf, w=1.0),
    ],
)
def test_invalid_imu_quaternion_invalidates_previous_yaw(node, quaternion):
    connect_with_telemetry(node, yaw=1.0)
    imu = Imu()
    imu.orientation = quaternion

    node.imu_callback(imu)
    node.publish_drone_position()

    assert not node._telemetry_ready()
    node.gps_pub.publish.assert_not_called()
    with pytest.raises(ValueError):
        node.quaternion_to_yaw(quaternion)


def test_unavailable_imu_orientation_invalidates_previous_yaw(node):
    connect_with_telemetry(node)
    imu = imu_attitude(yaw=1.0)
    imu.orientation_covariance[0] = -1.0

    node.imu_callback(imu)
    node.publish_drone_position()

    assert not node._telemetry_ready()
    node.gps_pub.publish.assert_not_called()


def test_small_quaternion_roundoff_is_normalized_before_extracting_yaw(node):
    quaternion = imu_attitude(yaw=1.0, roll=0.2, pitch=-0.3).orientation
    quaternion.x *= 1.005
    quaternion.y *= 1.005
    quaternion.z *= 1.005
    quaternion.w *= 1.005

    assert node.quaternion_to_yaw(quaternion) == pytest.approx(1.0, abs=1e-7)


@pytest.mark.parametrize(
    "field,value", [("latitude", math.nan), ("latitude", 91.0), ("longitude", -181.0)]
)
def test_invalid_gps_coordinates_invalidate_previous_fix(node, field, value):
    connect_with_telemetry(node)
    msg = gps_fix()
    setattr(msg, field, value)

    node.gps_callback(msg)

    assert not node._telemetry_ready()


def test_no_gps_fix_invalidates_previous_fix(node):
    connect_with_telemetry(node)
    msg = gps_fix()
    msg.status.status = NavSatStatus.STATUS_NO_FIX

    node.gps_callback(msg)

    assert not node._telemetry_ready()


@pytest.mark.parametrize("altitude", [math.nan, math.inf, -math.inf, 1e100, -1e100])
def test_invalid_relative_altitude_invalidates_previous_sample(node, altitude):
    connect_with_telemetry(node)

    node.relative_altitude_callback(Float64(data=altitude))

    assert not node._telemetry_ready()


@pytest.mark.parametrize(
    "field,value",
    [
        ("altitude", math.nan),
        ("altitude", math.inf),
        ("altitude", 1e100),
        ("altitude", -1e100),
        ("latitude", 91.0),
        ("longitude", -181.0),
    ],
)
def test_invalid_goal_is_not_published(node, field, value):
    connect_with_telemetry(node)
    node.new_position_callback(position_goal())
    goal = position_goal()
    setattr(goal, field, value)

    node.new_position_callback(goal)
    node.publish_setpoint()

    assert node.latest_setpoint is None
    node.setpoint_pub.publish.assert_not_called()


@pytest.mark.parametrize(
    "timestamp_field",
    ["gps_received_at", "relative_altitude_received_at", "imu_received_at"],
)
def test_sensor_timeout_clears_held_goal_and_prevents_replay(node, timestamp_field):
    connect_with_telemetry(node)
    node.new_position_callback(position_goal())
    setattr(node, timestamp_field, 97.9)

    node.publish_setpoint()
    assert node.latest_setpoint is None
    node.setpoint_pub.publish.assert_not_called()

    connect_with_telemetry(node)
    node.publish_setpoint()
    node.setpoint_pub.publish.assert_not_called()


def test_expired_goal_is_cleared_even_when_telemetry_remains_fresh(node, monkeypatch):
    connect_with_telemetry(node)
    node.new_position_callback(position_goal())
    monkeypatch.setattr(converter.time, "monotonic", lambda: 105.1)
    connect_with_telemetry(node)

    node.publish_setpoint()

    assert node.latest_setpoint is None
    node.setpoint_pub.publish.assert_not_called()


def test_held_goal_preserves_requested_heading_as_imu_changes(node):
    connect_with_telemetry(node, yaw=0.25)
    node.new_position_callback(position_goal())
    node.imu_callback(imu_attitude(yaw=1.25))

    node.publish_setpoint()

    assert node.current_yaw == pytest.approx(1.25)
    assert node.setpoint_pub.publish.call_args.args[0].yaw == pytest.approx(0.25)


def test_disconnect_requires_new_sensor_samples_and_never_replays_old_goal(node):
    connect_with_telemetry(node, yaw=0.5)
    node.new_position_callback(position_goal())

    node.state_callback(State(connected=False, mode=""))
    node.publish_drone_position()
    node.publish_setpoint()
    assert node.latest_setpoint is None
    assert node.current_yaw is None
    node.gps_pub.publish.assert_not_called()
    node.setpoint_pub.publish.assert_not_called()

    node.state_callback(State(connected=True, mode="GUIDED"))
    assert not node._telemetry_ready()
    connect_with_telemetry(node)
    node.publish_setpoint()
    node.setpoint_pub.publish.assert_not_called()


def test_takeoff_does_not_change_mode_or_arm_before_telemetry_is_ready(node):
    node.set_mode_callback = Mock()
    node.arm = Mock()
    node.create_client = Mock()
    node.call_service = Mock()

    node.takeoff(30.0)

    node.set_mode_callback.assert_not_called()
    node.arm.assert_not_called()
    node.create_client.assert_not_called()
    node.call_service.assert_not_called()
