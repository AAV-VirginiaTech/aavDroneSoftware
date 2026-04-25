"""
Safer MAVROS servo command helper for payload release.

Default behavior:
  1. Move servo output to start PWM
  2. Wait settle time
  3. Move to target PWM
  4. Hold target for hold_sec
  5. Move back to return PWM
  6. Wait settle time

Example using physical Kore/Pixhawk-style board labels:
  ros2 run <your_pkg> servo_command_node_fixed --output aux1 --start 1000 --target 1600 --return-pwm 1000 --hold 1.0

Example using raw ArduPilot SERVOx numbering:
  ros2 run <your_pkg> servo_command_node_fixed --servo 9 --start 1000 --target 1600 --return-pwm 1000 --hold 1.0

Output mapping:
  MAIN1-MAIN8 -> SERVO1-SERVO8
  AUX1-AUX6   -> SERVO9-SERVO14

Notes:
  - MAV_CMD_DO_SET_SERVO param1 is the ArduPilot SERVO instance number.
  - The physical board label, such as AUX1, is not always the same as the ArduPilot SERVOx number.
  - On Kore/Pixhawk-style carrier boards, AUX1 normally maps to SERVO9, AUX2 to SERVO10, etc.
  - On Cube/Core carrier boards, confirm the physical output maps to the SERVOx instance you pass here.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass

import rclpy
from mavros_msgs.srv import CommandLong
from rclpy.node import Node

MAV_CMD_DO_SET_SERVO = 183
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5


@dataclass(frozen=True)
class ServoSequenceConfig:
    servo_instance: int = 9
    physical_output: str = "aux1"
    start_pwm: float = 1000.0
    target_pwm: float = 1600.0
    return_pwm: float = 1000.0
    hold_sec: float = 1.0
    settle_sec: float = 0.5
    mode: str = "jump"
    ramp_step_us: float = 25.0
    ramp_delay_sec: float = 0.05
    min_pwm: float = 800.0
    max_pwm: float = 2200.0
    service_timeout_sec: float = 10.0
    command_timeout_sec: float = 5.0


class ServoCommandError(RuntimeError):
    """Raised when MAVROS/ArduPilot rejects or fails a servo command."""


def physical_output_to_servo_instance(output_label: str) -> int:
    """
    Convert a physical board output label into an ArduPilot SERVOx instance.

    Examples:
      main1 -> 1
      main8 -> 8
      aux1  -> 9
      aux6  -> 14

    This matches the common Cube/Pixhawk/Kore-style layout:
      MAIN1-MAIN8 -> SERVO1-SERVO8
      AUX1-AUX6   -> SERVO9-SERVO14
    """
    label = output_label.strip().lower().replace("_", "").replace("-", "")

    match = re.fullmatch(r"(main|aux)(\d+)", label)
    if not match:
        raise ValueError(
            f"Invalid output label '{output_label}'. Use labels like main1, main2, aux1, or aux6."
        )

    bank = match.group(1)
    number = int(match.group(2))

    if bank == "main":
        if not 1 <= number <= 8:
            raise ValueError("MAIN output must be in range main1 through main8")
        return number

    if bank == "aux":
        if not 1 <= number <= 6:
            raise ValueError("AUX output must be in range aux1 through aux6")
        return 8 + number

    # This should be unreachable because of the regex.
    raise ValueError(f"Unsupported output bank: {bank}")


def servo_instance_to_physical_output(servo_instance: int) -> str:
    """
    Convert an ArduPilot SERVOx instance back to the likely physical label.

    Examples:
      1  -> main1
      8  -> main8
      9  -> aux1
      14 -> aux6
    """
    if 1 <= servo_instance <= 8:
        return f"main{servo_instance}"
    if 9 <= servo_instance <= 14:
        return f"aux{servo_instance - 8}"
    return f"servo{servo_instance}"


class ServoCommandClient:
    def __init__(
        self,
        node: Node,
        service_name: str = "/mavros/cmd/command",
        timeout_sec: float = 10.0,
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

    def set_servo(
        self, servo_instance: int, pwm_us: float, timeout_sec: float = 5.0
    ) -> CommandLong.Response:
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
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for servo command response after {timeout_sec:.1f}s "
                f"(servo={servo_instance}, pwm={pwm_us})"
            )

        resp = future.result()
        if resp is None:
            raise ServoCommandError("No response from /mavros/cmd/command")

        physical_output = servo_instance_to_physical_output(servo_instance)
        self.node.get_logger().info(
            f"Set {physical_output.upper()} / SERVO{servo_instance} to {pwm_us:.0f} us -> "
            f"success={resp.success}, result={resp.result}"
        )

        # MAVROS success should normally be true and result should be MAV_RESULT_ACCEPTED.
        if not resp.success or resp.result != MAV_RESULT_ACCEPTED:
            raise ServoCommandError(
                f"Servo command rejected/failed: output={physical_output}, "
                f"servo={servo_instance}, pwm={pwm_us}, "
                f"success={resp.success}, result={resp.result}"
            )

        return resp

    def run_sequence(self, config: ServoSequenceConfig) -> None:
        validate_config(config)

        self.node.get_logger().info(
            "Starting servo sequence: "
            f"output={config.physical_output.upper()}, SERVO{config.servo_instance}, "
            f"start={config.start_pwm:.0f}, target={config.target_pwm:.0f}, "
            f"return={config.return_pwm:.0f}, hold={config.hold_sec:.2f}s, "
            f"mode={config.mode}"
        )

        self.set_servo(
            config.servo_instance, config.start_pwm, config.command_timeout_sec
        )
        time.sleep(config.settle_sec)

        if config.mode == "ramp":
            self._ramp_to(config, config.start_pwm, config.target_pwm)
        else:
            self.set_servo(
                config.servo_instance, config.target_pwm, config.command_timeout_sec
            )

        # This is the actual time the signal stays at target PWM after reaching target.
        time.sleep(config.hold_sec)

        if config.mode == "ramp":
            self._ramp_to(config, config.target_pwm, config.return_pwm)
        else:
            self.set_servo(
                config.servo_instance, config.return_pwm, config.command_timeout_sec
            )

        time.sleep(config.settle_sec)
        self.node.get_logger().info("Servo sequence complete")

    def _ramp_to(
        self, config: ServoSequenceConfig, start_pwm: float, end_pwm: float
    ) -> None:
        if start_pwm == end_pwm:
            self.set_servo(config.servo_instance, end_pwm, config.command_timeout_sec)
            return

        direction = 1.0 if end_pwm > start_pwm else -1.0
        step = abs(config.ramp_step_us) * direction
        value = start_pwm

        while (direction > 0 and value < end_pwm) or (
            direction < 0 and value > end_pwm
        ):
            self.set_servo(config.servo_instance, value, config.command_timeout_sec)
            time.sleep(config.ramp_delay_sec)
            value += step

        self.set_servo(config.servo_instance, end_pwm, config.command_timeout_sec)


def validate_config(config: ServoSequenceConfig) -> None:
    if config.servo_instance < 1:
        raise ValueError(
            "servo_instance must be >= 1. It should match the ArduPilot SERVOx output number."
        )

    for name, pwm in (
        ("start_pwm", config.start_pwm),
        ("target_pwm", config.target_pwm),
        ("return_pwm", config.return_pwm),
    ):
        if not config.min_pwm <= pwm <= config.max_pwm:
            raise ValueError(
                f"{name}={pwm} is outside the allowed range "
                f"[{config.min_pwm}, {config.max_pwm}] us"
            )

    if config.hold_sec < 0:
        raise ValueError("hold_sec must be non-negative")
    if config.settle_sec < 0:
        raise ValueError("settle_sec must be non-negative")
    if config.mode not in {"jump", "ramp"}:
        raise ValueError("mode must be either 'jump' or 'ramp'")
    if config.ramp_step_us <= 0:
        raise ValueError("ramp_step_us must be > 0")
    if config.ramp_delay_sec < 0:
        raise ValueError("ramp_delay_sec must be non-negative")


def estimate_active_time(config: ServoSequenceConfig) -> float:
    """Return approximate sequence duration excluding MAVROS service latency."""
    validate_config(config)
    total = config.settle_sec + config.hold_sec + config.settle_sec
    if config.mode == "ramp":
        up_steps = int(abs(config.target_pwm - config.start_pwm) // config.ramp_step_us)
        down_steps = int(
            abs(config.target_pwm - config.return_pwm) // config.ramp_step_us
        )
        total += (up_steps + down_steps) * config.ramp_delay_sec
    return total


def run_payload_drop_sequence(
    node: Node,
    output: str = "aux1",
    start_pwm: float = 1000.0,
    target_pwm: float = 1600.0,
    return_pwm: float = 1000.0,
    hold_sec: float = 1.0,
    mode: str = "jump",
) -> None:
    servo_instance = physical_output_to_servo_instance(output)
    config = ServoSequenceConfig(
        servo_instance=servo_instance,
        physical_output=output.strip().lower(),
        start_pwm=start_pwm,
        target_pwm=target_pwm,
        return_pwm=return_pwm,
        hold_sec=hold_sec,
        mode=mode.strip().lower(),
    )
    client = ServoCommandClient(node, timeout_sec=config.service_timeout_sec)
    client.run_sequence(config)


class ServoCommander(Node):
    def __init__(self, service_timeout_sec: float = 10.0):
        super().__init__("servo_commander")
        self.command_client = ServoCommandClient(self, timeout_sec=service_timeout_sec)

    def run_sequence(self, config: ServoSequenceConfig) -> None:
        self.command_client.run_sequence(config)


def parse_args(argv: list[str]) -> ServoSequenceConfig:
    parser = argparse.ArgumentParser(
        description="Command a MAVROS/ArduPilot servo output for payload release."
    )

    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        "--output",
        type=str,
        default="aux1",
        help=(
            "Physical board output label, such as main1, main2, aux1, or aux6. "
            "Default: aux1. Kore/Pixhawk-style mapping is MAIN1-MAIN8 -> SERVO1-SERVO8 "
            "and AUX1-AUX6 -> SERVO9-SERVO14."
        ),
    )
    output_group.add_argument(
        "--servo",
        type=int,
        default=None,
        help=(
            "Raw ArduPilot SERVOx instance number. This bypasses physical label conversion. "
            "Example: --servo 9 is usually AUX1."
        ),
    )

    parser.add_argument(
        "--start",
        type=float,
        default=1000.0,
        help="Start PWM in microseconds, default: 1000",
    )
    parser.add_argument(
        "--target",
        type=float,
        default=1600.0,
        help="Release/target PWM in microseconds, default: 1600",
    )
    parser.add_argument(
        "--return-pwm",
        type=float,
        default=1000.0,
        help="Return PWM in microseconds, default: 1000",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=1.0,
        help="Seconds to hold target PWM after reaching target, default: 1.0",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to wait after start/return commands, default: 0.5",
    )
    parser.add_argument(
        "--mode",
        choices=("jump", "ramp"),
        default="jump",
        help="Move directly or ramp gradually",
    )
    parser.add_argument(
        "--ramp-step",
        type=float,
        default=25.0,
        help="Ramp step size in microseconds, default: 25",
    )
    parser.add_argument(
        "--ramp-delay",
        type=float,
        default=0.05,
        help="Delay between ramp steps in seconds, default: 0.05",
    )
    parser.add_argument(
        "--min-pwm",
        type=float,
        default=800.0,
        help="Minimum allowed PWM safety bound, default: 800",
    )
    parser.add_argument(
        "--max-pwm",
        type=float,
        default=2200.0,
        help="Maximum allowed PWM safety bound, default: 2200",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for MAVROS service, default: 10",
    )
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for each command response, default: 5",
    )

    # Use argv[1:] so ROS launch can pass normal command-line args after stripping ROS-specific args.
    parsed, unknown = parser.parse_known_args(argv[1:])
    if unknown:
        print(f"Ignoring unknown ROS arguments: {unknown}", file=sys.stderr)

    if parsed.servo is not None:
        servo_instance = parsed.servo
        physical_output = servo_instance_to_physical_output(servo_instance)
    else:
        physical_output = parsed.output.strip().lower()
        servo_instance = physical_output_to_servo_instance(physical_output)

    config = ServoSequenceConfig(
        servo_instance=servo_instance,
        physical_output=physical_output,
        start_pwm=parsed.start,
        target_pwm=parsed.target,
        return_pwm=parsed.return_pwm,
        hold_sec=parsed.hold,
        settle_sec=parsed.settle,
        mode=parsed.mode,
        ramp_step_us=parsed.ramp_step,
        ramp_delay_sec=parsed.ramp_delay,
        min_pwm=parsed.min_pwm,
        max_pwm=parsed.max_pwm,
        service_timeout_sec=parsed.service_timeout,
        command_timeout_sec=parsed.command_timeout,
    )
    validate_config(config)
    return config


def main(args=None) -> int:
    argv = sys.argv if args is None else args
    config = parse_args(argv)

    should_shutdown = False
    if not rclpy.ok():
        rclpy.init(args=argv)
        should_shutdown = True

    node = None
    try:
        node = ServoCommander(service_timeout_sec=config.service_timeout_sec)
        node.get_logger().info(
            f"Using physical output {config.physical_output.upper()} as SERVO{config.servo_instance}"
        )
        node.get_logger().info(
            f"Estimated sequence time excluding service latency: {estimate_active_time(config):.2f}s"
        )
        node.run_sequence(config)
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
