from __future__ import annotations


class StateMachine:
    class State:
        def __init__(self, name):
            self.name = name

        def enter(self):
            pass

        def update(self):
            pass

        def exit(self) -> StateMachine.State | None:
            pass

    def __init__(self):
        self.state: StateMachine.State | None = None
