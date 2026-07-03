import math
from types import SimpleNamespace
from typing import Any, cast
from unittest.mock import Mock

from aav_msgs.msg import DronePosition, NewDronePosition, TargetPosition
from aav_software.manavs_magic_code import (
    Cam,
    Craft,
    ManavsMagicCode,
    TargPos,
    _get_utm_crs_for_position,
    calc_targ_dist,
    calc_targ_loc,
)
from aav_software.mission import Mission
from aav_software.object_alignment_controller import (
    ArduPilotMode,
    OacState,
    ObjectAlignmentController,
)


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def info(self, _msg):
        return None

    def warning(self, _msg):
        return None


class FakeElapsed:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __gt__(self, other):
        return self.nanoseconds > other.nanoseconds

    def __lt__(self, other):
        return self.nanoseconds < other.nanoseconds


class FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeElapsed(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self, now_nanoseconds):
        self.now_nanoseconds = now_nanoseconds

    def now(self):
        return FakeTime(self.now_nanoseconds)


def make_drone_position(latitude, longitude, altitude):
    msg = DronePosition()
    msg.latitude = latitude
    msg.longitude = longitude
    msg.altitude = altitude
    msg.yaw = 0.0
    return msg


def test_calc_targ_dist_center_pixel_has_zero_offset():
    craft = Craft()
    craft.alt = 70.0
    craft.yaw = 0.0

    cam = Cam()
    targ = TargPos()
    targ.x_norm = 0.5
    targ.y_norm = 0.5

    targ, _ = calc_targ_dist(craft, targ, cam)

    assert abs(targ.x_dist) < 1e-9
    assert abs(targ.y_dist) < 1e-9


def test_calc_targ_dist_right_of_center_maps_to_north_at_yaw_zero():
    craft = Craft()
    craft.alt = 50.0
    craft.yaw = 0.0

    cam = Cam()
    targ = TargPos()
    targ.x_norm = 1.0
    targ.y_norm = 0.5

    targ, _ = calc_targ_dist(craft, targ, cam)

    assert abs(targ.x_dist) < 1e-9
    assert targ.y_dist < 0.0


def test_calc_targ_loc_zero_offset_preserves_lat_lon():
    craft = Craft()
    craft.lat = 37.2296
    craft.lon = -80.4139

    targ = TargPos()
    targ.x_dist = 0.0
    targ.y_dist = 0.0

    _, targ = calc_targ_loc(craft, targ)

    assert abs(targ.lat - craft.lat) < 1e-6
    assert abs(targ.lon - craft.lon) < 1e-6


def test_get_utm_crs_for_position_changes_with_location():
    virginia_crs = _get_utm_crs_for_position(37.2296, -80.4139)
    sydney_crs = _get_utm_crs_for_position(-33.8688, 151.2093)

    virginia_authority = virginia_crs.to_authority()
    sydney_authority = sydney_crs.to_authority()

    assert virginia_authority is not None
    assert sydney_authority is not None
    assert virginia_authority[1].startswith("326")
    assert sydney_authority[1].startswith("327")
    assert virginia_authority != sydney_authority


def test_update_targ_gps_clamps_and_publishes_target_position():
    node = ManavsMagicCode.__new__(ManavsMagicCode)
    node_any = cast(Any, node)
    node_any.craft = Craft()
    node_any.craft.lat = 37.2296
    node_any.craft.lon = -80.4139
    node_any.craft.alt = 40.0
    node_any.craft.yaw = 0.0
    node_any.targ_pos = TargPos()
    node_any.cam = Cam()
    node_any.image_topic = "/siyi_a8/image_raw"
    publisher = CapturingPublisher()
    node_any.publisher = publisher
    node_any.get_logger = lambda: FakeLogger()

    image_msg = SimpleNamespace(width=1280, height=720)
    node.update_camera_resolution(cast(Any, image_msg))

    detection = SimpleNamespace(
        class_name="Bullseye",
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=999999.0, y=-100.0))
        ),
    )
    msg = SimpleNamespace(detections=[detection])

    node.update_targ_gps(cast(Any, msg))

    assert node_any.targ_pos.x_norm == 1.0
    assert node_any.targ_pos.y_norm == 0.0
    assert len(publisher.messages) == 1
    assert publisher.messages[0].object_label == "Bullseye"


def test_update_camera_resolution_drives_target_normalization():
    node = ManavsMagicCode.__new__(ManavsMagicCode)
    node_any = cast(Any, node)
    node_any.craft = Craft()
    node_any.craft.lat = 37.2296
    node_any.craft.lon = -80.4139
    node_any.craft.alt = 40.0
    node_any.craft.yaw = 0.0
    node_any.targ_pos = TargPos()
    node_any.cam = Cam()
    node_any.image_topic = "/siyi_a8/image_raw"
    publisher = CapturingPublisher()
    node_any.publisher = publisher
    node_any.get_logger = lambda: FakeLogger()

    node.update_camera_resolution(cast(Any, SimpleNamespace(width=1280, height=720)))

    detection = SimpleNamespace(
        class_name="Bullseye",
        bbox=SimpleNamespace(
            center=SimpleNamespace(position=SimpleNamespace(x=640.0, y=360.0))
        ),
    )
    msg = SimpleNamespace(detections=[detection])

    node.update_targ_gps(cast(Any, msg))

    assert math.isclose(node_any.targ_pos.x_norm, 0.5)
    assert math.isclose(node_any.targ_pos.y_norm, 0.5)
    assert len(publisher.messages) == 1


def test_target_position_callback_requests_guided_mode_once_when_not_guided():
    controller = ObjectAlignmentController.__new__(ObjectAlignmentController)
    controller_any = cast(Any, controller)
    controller_any.state = OacState.SEEKING
    controller_any.current_mode = ArduPilotMode.AUTO
    controller_any.current_mission = Mission.PACKAGE_DELIVERY_CUASC.value
    controller_any.guided_mode_request_in_flight = False
    controller_any.send_new_mode = Mock()
    publisher = CapturingPublisher()
    controller_any.new_position_pub = publisher
    # Ensure startup delay has already elapsed for tests
    controller_any.startup_time = FakeTime(0)
    controller_any.get_clock = lambda: FakeClock(
        ObjectAlignmentController.STARTUP_DELAY.nanoseconds + 1
    )

    target = TargetPosition()
    target.object_label = "Bullseye"
    target.latitude = 37.0
    target.longitude = -80.0

    controller.target_position_callback(target)

    controller_any.send_new_mode.assert_called_once_with(ArduPilotMode.GUIDED)
    assert controller_any.guided_mode_request_in_flight is True
    assert len(publisher.messages) == 0


def test_target_position_callback_publishes_when_guided_and_bullseye():
    controller = ObjectAlignmentController.__new__(ObjectAlignmentController)
    controller_any = cast(Any, controller)
    controller_any.state = OacState.SEEKING
    controller_any.current_mission = Mission.PACKAGE_DELIVERY_CUASC.value
    controller_any.current_mode = ArduPilotMode.GUIDED
    controller_any.guided_mode_request_in_flight = False
    controller_any.current_gps_position = make_drone_position(37.2295, -80.4138, 22.5)
    controller_any.last_target_position = None
    controller_any.time_marker = FakeTime(0)
    # Ensure startup delay has already elapsed for tests
    controller_any.startup_time = FakeTime(0)
    controller_any.get_clock = lambda: FakeClock(
        ObjectAlignmentController.STARTUP_DELAY.nanoseconds + 1
    )
    publisher = CapturingPublisher()
    controller_any.new_position_pub = publisher
    controller_any.get_logger = lambda: FakeLogger()
    controller_any.startup_delay_ended = False
    controller_any.seen_target = False

    target = TargetPosition()
    target.object_label = "Bullseye"
    target.latitude = 37.2296
    target.longitude = -80.4139

    controller.target_position_callback(target)

    assert len(publisher.messages) == 1
    new_position = publisher.messages[0]
    assert isinstance(new_position, NewDronePosition)
    assert math.isclose(new_position.latitude, 37.2296)
    assert math.isclose(new_position.longitude, -80.4139)
    assert math.isclose(new_position.altitude, 22.5)
    assert controller_any.last_target_label == "Bullseye"
    assert controller_any.seen_target is True


def test_update_state_machine_seeking_to_aligned_descending():
    controller = ObjectAlignmentController.__new__(ObjectAlignmentController)
    controller_any = cast(Any, controller)
    controller_any.state = OacState.SEEKING
    controller_any.current_mission = Mission.PACKAGE_DELIVERY_CUASC.value
    controller_any.last_target_position = NewDronePosition()
    controller_any.current_gps_position = make_drone_position(37.0, -80.0, 20.0)
    controller_any.time_marker = FakeTime(0)
    # Set startup time and advance clock past both startup and seek durations
    controller_any.startup_time = FakeTime(0)
    controller_any.get_clock = lambda: FakeClock(
        ObjectAlignmentController.STARTUP_DELAY.nanoseconds
        + ObjectAlignmentController.SEEK_ALIGNMENT_DURATION.nanoseconds
        + 1
    )
    controller_any.get_logger = lambda: FakeLogger()
    controller_any.startup_delay_ended = False
    controller_any.seen_target = True

    controller.update_state_machine()

    assert controller.state == OacState.ALIGNED_DESCENDING


def test_update_state_machine_aligned_descending_to_final_descending():
    controller = ObjectAlignmentController.__new__(ObjectAlignmentController)
    controller_any = cast(Any, controller)
    controller_any.state = OacState.FINAL_DESCENDING
    controller_any.current_mission = Mission.PACKAGE_DELIVERY_CUASC.value
    controller_any.descent_alignment_altitude = 5.0
    controller_any.hardcoded_drop_altitude = 3.0
    controller_any.current_gps_position = make_drone_position(37.2295, -80.4138, 5.0)
    controller_any.time_marker = FakeTime(0)
    # Ensure startup delay has already elapsed for tests
    controller_any.startup_time = FakeTime(0)
    controller_any.get_clock = lambda: FakeClock(
        ObjectAlignmentController.STARTUP_DELAY.nanoseconds + 1
    )
    publisher = CapturingPublisher()
    controller_any.new_position_pub = publisher
    controller_any.get_logger = lambda: FakeLogger()
    controller_any.startup_delay_ended = False

    controller.update_state_machine()

    assert controller.state == OacState.FINAL_DESCENDING
    assert len(publisher.messages) == 1
    published = publisher.messages[0]
    assert math.isclose(published.latitude, 37.2295)
    assert math.isclose(published.longitude, -80.4138)
    assert math.isclose(published.altitude, 3.0)
