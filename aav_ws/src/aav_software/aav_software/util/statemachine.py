from __future__ import annotations

from rclpy.node import Node

from .timer import Timer


class StateMachine:
    class State(Node):
        def __init__(self, name):
            self.name = name
            self.timer = Timer(self.get_clock())

        def get_timer(self) -> Timer:
            return self.timer

        def enter(self):
            pass

        def update(self):
            pass

        def exit(self) -> StateMachine.State | None:
            pass

    def __init__(self):
        self.state: StateMachine.State | None = None
