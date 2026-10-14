"""Test double for PowerGuard. Off-Pi menu tests have no INA219; the real guard
correctly refuses to drive without one, so menu tests that only check routing
install this stand-in. Guard behaviour itself is tested in test_v9_1.py."""


class FakeGuard:
    instances = []

    def __init__(self):
        self.checks = 0
        self.closed = False
        self.trip_reason = None
        FakeGuard.instances.append(self)

    @classmethod
    def from_settings(cls, settings, **kwargs):
        return cls()

    def wait_ready(self, poll=None):
        if poll is not None:
            poll()
        return None

    def check(self, motors):
        self.checks += 1

    def summary(self):
        return None

    def close(self):
        self.closed = True
        return True
