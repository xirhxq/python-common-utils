# tests/python38/test_pid_simulated_time.py

from tests.common.test_pid_base import TestPIDBase


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


class TestPIDSimulatedTime(TestPIDBase.TestPIDBase):
    def setUp(self):
        self.simulated_time = SimulatedTime()
        super().setUp()

    def get_time_func(self):
        return self.simulated_time.get_time

    def reset_time(self):
        self.simulated_time.time = 0.0

    def advance_time(self, delta):
        self.simulated_time.advance_time(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
