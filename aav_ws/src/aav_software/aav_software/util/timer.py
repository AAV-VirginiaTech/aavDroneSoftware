from rclpy.clock import Clock


class Timer:
    def __init__(self, clock: Clock):
        self.start = clock.now()
        self.clock = clock

    def elapsed(self):
        if self.start is None:
            return 0
        return self.clock.now() - self.start
