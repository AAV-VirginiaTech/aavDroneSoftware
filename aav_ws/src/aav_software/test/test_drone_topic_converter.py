"""Regression coverage for the MAVROS relative-altitude command path."""

import math
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import Mock

import pytest
from aav_msgs.msg import Mode, NewDronePosition
from aav_software import topic_converter_for_drone as converter
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


@pytest.fixture
def node(monkeypatch):
    instance = cast(Any, converter.TopicConverter.__new__(converter.TopicConverter))
    instance.current_latitude = None
    instance.current_longitude = None
    instance.current_relative_altitude = None
    instance.current_yaw = 0.0
    instance.current_mode = None
    instance.last_setpoint_publish_time = 0.0
    instance.setpoint_rate_limit_interval = 5.0
    instance.mode_pub = Mock()
    instance.gps_pub = Mock()
    instance.setpoint_pub = Mock()
    instance.get_logger = Mock(return_value=Mock())
    instance.get_clock = Mock(
        return_value=SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))
        )
    )
    monkeypatch.setattr(converter.time, "time", lambda: 100.0)
    return instance


def gps_fix(latitude=37.2296, longitude=-80.4139, altitude=530.0):
    return NavSatFix(latitude=latitude, longitude=longitude, altitude=altitude)


def goal(altitude=20.0):
    return NewDronePosition(latitude=37.2297, longitude=-80.4138, altitude=altitude)


def pose(yaw):
    msg = PoseStamped()
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_mavros_topics_qos_and_raw_setpoint_publisher(monkeypatch):
    monkeypatch.setattr(converter.Node, "__init__", lambda self, name: None)
    monkeypatch.setattr(converter.Node, "get_logger", Mock(return_value=Mock()))
    subscriptions, publishers, timers = Mock(), Mock(), Mock()
    monkeypatch.setattr(converter.Node, "create_subscription", subscriptions)
    monkeypatch.setattr(converter.Node, "create_publisher", publishers)
    monkeypatch.setattr(converter.Node, "create_timer", timers)

    converter.TopicConverter()

    by_topic = {call.args[1]: call.args for call in subscriptions.call_args_list}
    assert set(by_topic) == {
        "/mavros/state",
        "/mavros/global_position/global",
        "/mavros/global_position/rel_alt",
        "/mavros/local_position/pose",
        "/AAV/set_mode",
        "/AAV/send_new_position",
    }
    for topic in (
        "/mavros/global_position/global",
        "/mavros/global_position/rel_alt",
        "/mavros/local_position/pose",
    ):
        assert by_topic[topic][3].reliability == ReliabilityPolicy.BEST_EFFORT
        assert by_topic[topic][3].depth == 10
    publishers.assert_any_call(GlobalPositionTarget, "/mavros/setpoint_raw/global", 10)
    timers.assert_not_called()


def test_current_position_uses_mavros_relative_altitude(node):
    node.pose_callback(pose(0.5))
    node.gps_callback(gps_fix())
    node.gps_pub.publish.assert_not_called()

    node.relative_altitude_callback(Float64(data=12.5))
    position = node.gps_pub.publish.call_args.args[0]
    assert position.latitude == pytest.approx(37.2296)
    assert position.longitude == pytest.approx(-80.4139)
    assert position.altitude == pytest.approx(12.5)
    assert position.yaw == pytest.approx(0.5)

    node.relative_altitude_callback(Float64(data=14.0))
    assert node.gps_pub.publish.call_args.args[0].altitude == pytest.approx(14.0)


def test_position_command_uses_relative_altitude_frame_and_direct_target_altitude(
    node,
):
    node.pose_callback(pose(math.pi / 2))
    node.new_position_callback(goal(altitude=20.0))

    target = node.setpoint_pub.publish.call_args.args[0]
    assert isinstance(target, GlobalPositionTarget)
    assert target.header.frame_id == "map"
    assert target.header.stamp.sec == 123
    assert target.coordinate_frame == GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    assert target.type_mask == node.POSITION_AND_YAW_MASK
    assert target.latitude == pytest.approx(37.2297)
    assert target.longitude == pytest.approx(-80.4138)
    assert target.altitude == pytest.approx(20.0)
    assert target.yaw == pytest.approx(math.pi / 2)


def test_position_commands_are_rate_limited_to_five_seconds(node, monkeypatch):
    node.new_position_callback(goal())
    node.setpoint_pub.publish.assert_called_once()

    monkeypatch.setattr(converter.time, "time", lambda: 104.9)
    node.new_position_callback(goal(altitude=10.0))
    node.setpoint_pub.publish.assert_called_once()

    monkeypatch.setattr(converter.time, "time", lambda: 105.0)
    node.new_position_callback(goal(altitude=10.0))
    assert node.setpoint_pub.publish.call_count == 2


def test_position_command_does_not_require_current_telemetry(node):
    node.new_position_callback(goal())
    node.setpoint_pub.publish.assert_called_once()


def test_yaw_is_captured_when_goal_arrives(node):
    node.pose_callback(pose(0.5))
    node.new_position_callback(goal())
    target = node.setpoint_pub.publish.call_args.args[0]
    assert target.yaw == pytest.approx(0.5)


def test_state_mode_is_forwarded(node):
    node.state_callback(State(mode="GUIDED"))
    node.mode_pub.publish.assert_called_once()
    assert node.mode_pub.publish.call_args.args[0].mode == 4


def test_loiter_mode_guard_is_preserved(node):
    node.state_callback(State(mode="LOITER"))
    node.create_client = Mock()
    node.set_mode_callback(Mode(mode=4))
    node.create_client.assert_not_called()
    node.get_logger().warn.assert_called_once_with("Cannot switch out of LOITER")
