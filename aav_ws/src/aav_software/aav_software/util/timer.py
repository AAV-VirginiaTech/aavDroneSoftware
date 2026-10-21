from rclpy.clock import Clock
from rclpy.duration import Duration


class Timer:
    def __init__(self, clock: Clock):
        self.start = clock.now()
        self.clock = clock

    def now(self):
        return self.clock.now()

    def elapsed(self) -> Duration:
        result = self.clock.now() - self.start
        assert isinstance(result, Duration)
        return result

    def reset(self):
        self.start = self.clock.now()
