"""
Simple MAVROS payload release helper.

This node intentionally supports only 3 user-facing parameters:

  1. openandclose: boolean
     - true:  send close PWM, then open PWM, then close PWM again
     - false: send close PWM, then open PWM and leave it there

  2. pwm: float
     - the open/release PWM value

  3. aux_port: int
     - physical AUX port number on the board, 1 through 6
     - converted internally to ArduPilot SERVOx:
         AUX1 -> SERVO9
         AUX2 -> SERVO10
         AUX3 -> SERVO11
         AUX4 -> SERVO12
         AUX5 -> SERVO13
         AUX6 -> SERVO14

Example:
  ros2 run aav_software cuasc_drop true 1900 5

This means:
  openandclose = true
  open PWM     = 1900 us
  physical AUX = AUX5
  ArduPilot    = SERVO13
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import rclpy
from mavros_msgs.srv import CommandLong
from rclpy.node import Node

MAV_CMD_DO_SET_SERVO = 183
MAV_RESULT_ACCEPTED = 0

CLOSE_PWM = 1000.0
SETTLE_SEC = 0.5
OPEN_HOLD_SEC = 1.0
MIN_PWM = 800.0
MAX_PWM = 2200.0
SERVICE_TIMEOUT_SEC = 10.0
COMMAND_TIMEOUT_SEC = 5.0


@dataclass(frozen=True)
class PayloadReleaseConfig:
    openandclose: bool
    open_pwm: float
    aux_port: int
    servo_instance: int


class ServoCommandError(RuntimeError):
    """Raised when MAVROS/ArduPilot rejects or fails a servo command."""


def aux_port_to_servo_instance(aux_port: int) -> int:
    """
    Convert physical AUX port label into ArduPilot SERVOx number.

    AUX1 -> SERVO9
    AUX2 -> SERVO10
    AUX3 -> SERVO11
    AUX4 -> SERVO12
    AUX5 -> SERVO13
    AUX6 -> SERVO14
    """
    if not 1 <= aux_port <= 6:
        raise ValueError("aux_port must be in range 1 through 6")

    return 8 + aux_port


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()

    if normalized in {"true", "t", "yes", "y", "1"}:
        return True

    if normalized in {"false", "f", "no", "n", "0"}:
        return False

    raise argparse.ArgumentTypeError("openandclose must be true or false")


def validate_config(config: PayloadReleaseConfig) -> None:
    if not MIN_PWM <= config.open_pwm <= MAX_PWM:
        raise ValueError(
            f"pwm={config.open_pwm} is outside the allowed range [{MIN_PWM}, {MAX_PWM}] us"
        )

    if not 1 <= config.aux_port <= 6:
        raise ValueError("aux_port must be in range 1 through 6")


class ServoCommandClient:
    def __init__(
        self,
        node: Node,
        service_name: str = "/mavros/cmd/command",
        timeout_sec: float = SERVICE_TIMEOUT_SEC,
    ):
        self.node = node
        self.cli = self.node.create_client(CommandLong, service_name)

        self.node.get_logger().info(f"Waiting for {service_name} service...")
        deadline = time.monotonic() + timeout_sec

        while not self.cli.wait_for_service(timeout_sec=1.0):
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for {service_name} after {timeout_sec:.1f}s"
                )
            self.node.get_logger().info("Service not available yet...")

    def set_servo(self, servo_instance: int, pwm_us: float) -> CommandLong.Response:
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_SET_SERVO
        req.confirmation = 0
        req.param1 = float(servo_instance)
        req.param2 = float(pwm_us)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=COMMAND_TIMEOUT_SEC,
        )

        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for servo command response after {COMMAND_TIMEOUT_SEC:.1f}s "
                f"(servo={servo_instance}, pwm={pwm_us})"
            )

        resp = future.result()
        if resp is None:
            raise ServoCommandError("No response from /mavros/cmd/command")

        aux_port = servo_instance - 8
        self.node.get_logger().info(
            f"Set AUX{aux_port} / SERVO{servo_instance} to {pwm_us:.0f} us -> "
            f"success={resp.success}, result={resp.result}"
        )

        if not resp.success or resp.result != MAV_RESULT_ACCEPTED:
            raise ServoCommandError(
                f"Servo command rejected/failed: AUX{aux_port}, SERVO{servo_instance}, "
                f"pwm={pwm_us}, success={resp.success}, result={resp.result}"
            )

        return resp

    def run_payload_release(self, config: PayloadReleaseConfig) -> None:
        validate_config(config)

        self.node.get_logger().info(
            f"Using AUX{config.aux_port} as SERVO{config.servo_instance}"
        )

        self.set_servo(config.servo_instance, CLOSE_PWM)
        time.sleep(SETTLE_SEC)

        self.set_servo(config.servo_instance, config.open_pwm)

        if config.openandclose:
            time.sleep(OPEN_HOLD_SEC)
            self.set_servo(config.servo_instance, CLOSE_PWM)
            time.sleep(SETTLE_SEC)
            self.node.get_logger().info("Payload release opened and closed")
        else:
            self.node.get_logger().info("Payload release opened and left open")


class PayloadReleaseNode(Node):
    def __init__(self):
        super().__init__("payload_release")
        self.command_client = ServoCommandClient(self)

    def run_payload_release(self, config: PayloadReleaseConfig) -> None:
        self.command_client.run_payload_release(config)


def parse_args(argv: list[str]) -> PayloadReleaseConfig:
    parser = argparse.ArgumentParser(
        description="Command a physical AUX output for payload release."
    )

    parser.add_argument(
        "openandclose",
        type=parse_bool,
        help="true: close -> open -> close. false: close -> open and stay open.",
    )
    parser.add_argument(
        "pwm",
        type=float,
        help="Open/release PWM value in microseconds, usually 1800-2000.",
    )
    parser.add_argument(
        "aux_port",
        type=int,
        help="Physical AUX port number on the board, 1 through 6.",
    )

    parsed, unknown = parser.parse_known_args(argv[1:])
    if unknown:
        print(f"Ignoring unknown ROS arguments: {unknown}", file=sys.stderr)

    servo_instance = aux_port_to_servo_instance(parsed.aux_port)

    config = PayloadReleaseConfig(
        openandclose=parsed.openandclose,
        open_pwm=parsed.pwm,
        aux_port=parsed.aux_port,
        servo_instance=servo_instance,
    )
    validate_config(config)
    return config


def run_payload_drop_sequence(
    node: Node,
    openandclose: bool,
    pwm: float,
    aux_port: int,
) -> None:
    servo_instance = aux_port_to_servo_instance(aux_port)
    config = PayloadReleaseConfig(
        openandclose=openandclose,
        open_pwm=pwm,
        aux_port=aux_port,
        servo_instance=servo_instance,
    )

    client = ServoCommandClient(node)
    client.run_payload_release(config)


def main(args=None) -> int:
    argv = sys.argv if args is None else args
    config = parse_args(argv)

    should_shutdown = False
    if not rclpy.ok():
        rclpy.init(args=argv)
        should_shutdown = True

    node = None
    try:
        node = PayloadReleaseNode()
        node.run_payload_release(config)
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI should log and return nonzero for any failure.
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if should_shutdown:
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
