from __future__ import annotations

from rclpy.clock import Clock

from .timer import Timer


class StateMachine:
    class State:
        def __init__(self, name: str):
            self.name: str = name
            self.finished: bool = False
            self.next_state: StateMachine.State | None = None

        def get_name(self) -> str:
            return self.name

        def enter(self):
            pass

        def update(self):
            pass

        def exit(self):
            pass

        def mark_finished(self, next_state: StateMachine.State | None = None):
            self.finished = True
            self.next_state = next_state

        def is_finished(self) -> bool:
            return self.finished

        def get_next_state(self) -> StateMachine.State | None:
            return self.next_state

    def __init__(self, clock: Clock):
        self.current_state: StateMachine.State | None = None
        self.state_timer = Timer(clock)

    def start(self, initial_state: StateMachine.State):
        self.current_state = initial_state

        self.state_timer.reset()
        self.current_state.enter()

    def update(self):
        if self.current_state is None:
            return

        self.current_state.update()
        if not self.current_state.is_finished():
            return

        # cleanup (if necessary)
        self.current_state.exit()

        # set current state to next state
        self.current_state = self.current_state.get_next_state()
        if self.current_state is not None:
            self.state_timer.reset()
            self.current_state.enter()

    def get_current_state(self) -> StateMachine.State | None:
        return self.current_state

    def get_state_time(self):
        return self.state_timer.elapsed()
