import sys
import time

import rclpy
from mavros_msgs.srv import CommandLong
from rclpy.node import Node


class ServoCommandClient:
    def __init__(self, node: Node, service_name: str = "/mavros/cmd/command"):
        self.node = node
        self.cli = self.node.create_client(CommandLong, service_name)

        self.node.get_logger().info(f"Waiting for {service_name} service...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available yet...")

    def set_servo(self, servo_instance: int, pwm_us: float, confirmation: int = 0):
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.confirmation = confirmation
        req.param1 = float(servo_instance)  # Servo instance number
        req.param2 = float(pwm_us)  # PWM in microseconds
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        resp = future.result()
        if resp is None:
            raise RuntimeError("No response from /mavros/cmd/command")

        self.node.get_logger().info(
            f"Set servo {servo_instance} to {pwm_us} us -> "
            f"success={resp.success}, result={resp.result}"
        )
        return resp

    def run_sequence(
        self,
        servo_instance: int,
        start_pwm: float = 800.0,
        target_pwm: float = 1600.0,
        return_pwm: float = 800.0,
        hold_sec: float = 1.0,
        use_ramp: bool = False,
        ramp_step_us: float = 25.0,
        ramp_delay_sec: float = 0.05,
    ):
        # Go to start position
        self.set_servo(servo_instance, start_pwm)
        time.sleep(0.5)

        if use_ramp:
            value = start_pwm
            while value < target_pwm:
                self.set_servo(servo_instance, value)
                time.sleep(ramp_delay_sec)
                value += ramp_step_us
            self.set_servo(servo_instance, target_pwm)
        else:
            self.set_servo(servo_instance, target_pwm)

        time.sleep(hold_sec)

        if use_ramp:
            value = target_pwm
            while value > return_pwm:
                self.set_servo(servo_instance, value)
                time.sleep(ramp_delay_sec)
                value -= ramp_step_us
            self.set_servo(servo_instance, return_pwm)
        else:
            self.set_servo(servo_instance, return_pwm)

        time.sleep(0.5)


def run_payload_drop_sequence(
    node: Node,
    servo_instance: int = 2,
    start_pwm: float = 800.0,
    target_pwm: float = 1600.0,
    return_pwm: float = 800.0,
    hold_sec: float = 1.0,
    mode: str = "jump",
):
    client = ServoCommandClient(node)
    client.run_sequence(
        servo_instance=servo_instance,
        start_pwm=start_pwm,
        target_pwm=target_pwm,
        return_pwm=return_pwm,
        hold_sec=hold_sec,
        use_ramp=(mode.strip().lower() == "ramp"),
        ramp_step_us=25.0,
        ramp_delay_sec=0.05,
    )


class ServoCommander(Node):
    def __init__(self):
        super().__init__("servo_commander")
        self.command_client = ServoCommandClient(self)

    def run_sequence(
        self,
        servo_instance: int,
        start_pwm: float = 800.0,
        target_pwm: float = 1600.0,
        return_pwm: float = 800.0,
        hold_sec: float = 1.0,
        use_ramp: bool = False,
        ramp_step_us: float = 25.0,
        ramp_delay_sec: float = 0.05,
    ):
        self.command_client.run_sequence(
            servo_instance=servo_instance,
            start_pwm=start_pwm,
            target_pwm=target_pwm,
            return_pwm=return_pwm,
            hold_sec=hold_sec,
            use_ramp=use_ramp,
            ramp_step_us=ramp_step_us,
            ramp_delay_sec=ramp_delay_sec,
        )


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
        should_shutdown = True
    else:
        should_shutdown = False

    # Optional command-line usage:
    # python3 servo_command_node.py <servo_instance> <start_pwm> <target_pwm> <return_pwm> <hold_sec> <jump|ramp>
    servo_instance = 2
    start_pwm = 800.0
    target_pwm = 1600.0
    return_pwm = 800.0
    hold_sec = 1.0
    mode = "jump"

    if len(sys.argv) > 1:
        servo_instance = int(sys.argv[1])
    if len(sys.argv) > 2:
        start_pwm = float(sys.argv[2])
    if len(sys.argv) > 3:
        target_pwm = float(sys.argv[3])
    if len(sys.argv) > 4:
        return_pwm = float(sys.argv[4])
    if len(sys.argv) > 5:
        hold_sec = float(sys.argv[5])
    if len(sys.argv) > 6:
        mode = sys.argv[6].strip().lower()

    node = ServoCommander()

    try:
        node.run_sequence(
            servo_instance=servo_instance,
            start_pwm=start_pwm,
            target_pwm=target_pwm,
            return_pwm=return_pwm,
            hold_sec=hold_sec,
            use_ramp=(mode == "ramp"),
            ramp_step_us=25.0,
            ramp_delay_sec=0.05,
        )
    finally:
        node.destroy_node()
        if should_shutdown:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
