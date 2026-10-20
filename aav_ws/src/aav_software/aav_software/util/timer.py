from rclpy.clock import Clock


class Timer:
    def __init__(self, clock: Clock):
        self.start = clock.now()
        self.clock = clock

    def now(self):
        return self.clock.now()

    def elapsed(self):
        if self.start is None:
            return 0
        return self.clock.now() - self.start

    def reset(self):
        self.start = self.clock.now()