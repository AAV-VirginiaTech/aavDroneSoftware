"""Regression coverage for the original command path with corrected AMSL altitude."""

import math
import subprocess
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import Mock

import pytest
from aav_msgs.msg import Mode, NewDronePosition
from aav_software import topic_converter_for_drone as converter
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import NavSatFix


@pytest.fixture
def geoid_run(monkeypatch):
    run = Mock(return_value=subprocess.CompletedProcess([], 0, stdout="30.0\n"))
    monkeypatch.setattr(converter.subprocess, "run", run)
    return run


@pytest.fixture
def node(monkeypatch, geoid_run):
    instance = cast(Any, converter.TopicConverter.__new__(converter.TopicConverter))
    instance.home_altitude = None
    instance.home_latitude = None
    instance.home_longitude = None
    instance.home_altitude_amsl = None
    instance.current_latitude = None
    instance.current_longitude = None
    instance.current_yaw = 0.0
    instance.current_mode = None
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
    monkeypatch.setattr(converter.time, "time", lambda: 100.0)
    return instance


def gps_fix(altitude=530.0, latitude=37.2296, longitude=-80.4139):
    return NavSatFix(latitude=latitude, longitude=longitude, altitude=altitude)


def goal(altitude=20.0):
    return NewDronePosition(latitude=37.2297, longitude=-80.4138, altitude=altitude)


def pose(yaw):
    msg = PoseStamped()
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_original_topics_qos_and_timer_are_restored(monkeypatch):
    monkeypatch.setattr(converter.Node, "__init__", lambda self, name: None)
    monkeypatch.setattr(converter.Node, "get_logger", Mock(return_value=Mock()))
    monkeypatch.setattr(
        converter.subprocess,
        "run",
        Mock(return_value=subprocess.CompletedProcess([], 0, stdout="-30.0\n")),
    )
    subscriptions, publishers, timers = Mock(), Mock(), Mock()
    monkeypatch.setattr(converter.Node, "create_subscription", subscriptions)
    monkeypatch.setattr(converter.Node, "create_publisher", publishers)
    monkeypatch.setattr(converter.Node, "create_timer", timers)

    converter.TopicConverter()

    by_topic = {call.args[1]: call.args for call in subscriptions.call_args_list}
    assert set(by_topic) == {
        "/mavros/state",
        "/mavros/global_position/global",
        "/mavros/local_position/pose",
        "/AAV/set_mode",
        "/AAV/send_new_position",
    }
    for topic in ("/mavros/global_position/global", "/mavros/local_position/pose"):
        assert by_topic[topic][3].reliability == ReliabilityPolicy.BEST_EFFORT
        assert by_topic[topic][3].depth == 10
    publishers.assert_any_call(GeoPoseStamped, "/mavros/setpoint_position/global", 10)
    assert len(timers.call_args_list) == 1
    assert timers.call_args.args[0] == 0.2


def test_startup_geoid_check_logs_success(node, geoid_run):
    node.check_geoid_eval()
    geoid_run.assert_called_with(
        ["GeoidEval", "-n", "egm96-5"],
        input="0 0\n",
        text=True,
        capture_output=True,
        check=True,
        timeout=1.0,
    )
    assert "Startup geoid check passed" in node.get_logger().info.call_args.args[0]


@pytest.mark.parametrize(
    "failure,expected",
    [
        (FileNotFoundError(), "GeoidEval executable not found"),
        (
            subprocess.CalledProcessError(1, "GeoidEval", stderr="egm96-5.pgm missing"),
            "could not load egm96-5",
        ),
    ],
)
def test_startup_geoid_check_logs_actionable_failure(
    node, geoid_run, failure, expected
):
    geoid_run.side_effect = failure

    node.check_geoid_eval()

    assert expected in node.get_logger().error.call_args.args[0]
    assert (
        "Position commands requiring altitude conversion will be rejected"
        in (node.get_logger().error.call_args.args[0])
    )


def test_receiving_position_uses_first_fix_and_local_pose_without_geoid_lookup(
    node, geoid_run
):
    node.pose_callback(pose(0.5))
    node.gps_callback(gps_fix())
    assert node.gps_pub.publish.call_args.args[0].altitude == 0.0

    node.gps_callback(gps_fix(altitude=550.0))

    position = node.gps_pub.publish.call_args.args[0]
    assert position.latitude == pytest.approx(37.2296)
    assert position.longitude == pytest.approx(-80.4139)
    assert position.altitude == pytest.approx(20.0)
    assert position.yaw == pytest.approx(0.5)
    assert node.home_altitude == 530.0
    geoid_run.assert_not_called()


@pytest.mark.parametrize("geoid_height", [-30.0, 0.0, 30.0])
def test_global_target_corrects_ellipsoid_to_amsl_with_signed_geoid_height(
    node, geoid_run, geoid_height
):
    geoid_run.return_value.stdout = str(geoid_height)
    node.gps_callback(gps_fix())
    node.pose_callback(pose(math.pi / 2))

    node.new_position_callback(goal())
    node.publish_setpoint()

    target = node.setpoint_pub.publish.call_args.args[0]
    assert isinstance(target, GeoPoseStamped)
    assert target.header.frame_id == "map"
    assert target.header.stamp.sec == 123
    assert target.pose.position.latitude == pytest.approx(37.2297)
    assert target.pose.position.longitude == pytest.approx(-80.4138)
    assert target.pose.position.altitude == pytest.approx(550.0 - geoid_height)
    assert target.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert target.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_home_lookup_uses_first_location_even_if_drone_moves_before_first_goal(
    node, geoid_run
):
    node.gps_callback(gps_fix())
    node.gps_callback(gps_fix(altitude=560.0, latitude=38.0, longitude=-81.0))
    node.new_position_callback(goal())
    assert node.latest_setpoint.pose.position.altitude == pytest.approx(520.0)
    geoid_run.assert_called_once_with(
        ["GeoidEval", "-n", "egm96-5"],
        input="37.2296 -80.4139\n",
        text=True,
        capture_output=True,
        check=True,
        timeout=1.0,
    )


def test_successful_conversion_is_cached_across_goals_and_timer_ticks(node, geoid_run):
    node.gps_callback(gps_fix())
    node.new_position_callback(goal())
    for _ in range(3):
        node.publish_setpoint()
    node.gps_callback(gps_fix(altitude=570.0))
    node.new_position_callback(goal(altitude=10.0))
    node.publish_setpoint()

    assert node.latest_setpoint.pose.position.altitude == pytest.approx(510.0)
    geoid_run.assert_called_once()


def test_zero_startup_altitude_is_kept_as_reference(node):
    node.gps_callback(gps_fix(altitude=0.0))
    node.gps_callback(gps_fix(altitude=15.0))
    node.new_position_callback(goal())
    assert node.home_altitude == 0.0
    assert node.gps_pub.publish.call_args.args[0].altitude == pytest.approx(15.0)
    assert node.latest_setpoint.pose.position.altitude == pytest.approx(-10.0)


def test_command_needs_no_imu_connection_or_freshness_gate(node, monkeypatch):
    node.gps_callback(gps_fix())
    monkeypatch.setattr(converter.time, "time", lambda: 1000.0)
    node.new_position_callback(goal())
    node.publish_setpoint()
    assert node.latest_setpoint.pose.orientation.z == 0.0
    assert node.latest_setpoint.pose.orientation.w == 1.0
    node.setpoint_pub.publish.assert_called_once()


def test_heading_is_captured_when_goal_arrives(node):
    node.gps_callback(gps_fix())
    node.pose_callback(pose(0.5))
    node.new_position_callback(goal())
    node.pose_callback(pose(1.0))
    node.publish_setpoint()
    assert node.latest_setpoint.pose.orientation.z == pytest.approx(math.sin(0.25))


def test_original_five_second_window_and_new_goal_restart(node, monkeypatch):
    node.gps_callback(gps_fix())
    node.new_position_callback(goal())
    monkeypatch.setattr(converter.time, "time", lambda: 105.0)
    node.publish_setpoint()
    node.setpoint_pub.publish.assert_called_once()
    monkeypatch.setattr(converter.time, "time", lambda: 105.1)
    node.publish_setpoint()
    node.setpoint_pub.publish.assert_called_once()
    node.new_position_callback(goal(altitude=10.0))
    node.publish_setpoint()
    assert node.setpoint_pub.publish.call_count == 2


def test_goal_before_first_gps_does_not_guess_absolute_altitude(node, geoid_run):
    node.new_position_callback(goal())
    node.publish_setpoint()
    node.setpoint_pub.publish.assert_not_called()
    geoid_run.assert_not_called()
    assert "first GPS reading" in node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize(
    "failure", [FileNotFoundError(), subprocess.TimeoutExpired("GeoidEval", 1.0)]
)
def test_failed_lookup_logs_and_can_retry_without_affecting_received_position(
    node, geoid_run, failure
):
    node.gps_callback(gps_fix())
    geoid_run.side_effect = failure
    node.new_position_callback(goal())
    assert node.latest_setpoint is None
    assert node.home_altitude_amsl is None
    assert "geographiclib-tools" in node.get_logger().error.call_args.args[0]
    node.gps_callback(gps_fix(altitude=535.0))
    assert node.gps_pub.publish.call_args.args[0].altitude == pytest.approx(5.0)
    geoid_run.side_effect = None
    node.new_position_callback(goal())
    assert node.latest_setpoint.pose.position.altitude == pytest.approx(520.0)


def test_missing_geoid_dataset_logs_tool_error_and_install_command(node, geoid_run):
    node.gps_callback(gps_fix())
    details = "File /usr/share/GeographicLib/geoids/egm96-5.pgm not readable"
    geoid_run.side_effect = subprocess.CalledProcessError(
        1, ["GeoidEval", "-n", "egm96-5"], stderr=details
    )

    node.new_position_callback(goal())
    node.publish_setpoint()

    log = node.get_logger().error.call_args
    assert details in log.args[0]
    assert "sudo geographiclib-get-geoids egm96-5" in log.args[0]
    assert "Position command not sent" in log.args[0]
    assert log.kwargs["throttle_duration_sec"] == 5.0
    node.setpoint_pub.publish.assert_not_called()


@pytest.mark.parametrize("output", ["nan", "inf", "bad data", ""])
def test_invalid_geoid_output_is_not_cached_or_sent(node, geoid_run, output):
    node.gps_callback(gps_fix())
    geoid_run.return_value.stdout = output
    node.new_position_callback(goal())
    assert node.home_altitude_amsl is None
    assert node.latest_setpoint is None


def test_separately_added_loiter_mode_guard_is_preserved(node):
    node.state_callback(State(mode="LOITER"))
    node.create_client = Mock()
    node.set_mode_callback(Mode(mode=4))
    node.create_client.assert_not_called()
    node.get_logger().warn.assert_called_once_with("Cannot switch out of LOITER")
