# tests/python38/test_pid_simulated_time.py

import unittest
import logging
from core.PID import PID


logging.basicConfig(
    level=logging.CRITICAL, format="%(asctime)s - %(levelname)s - %(message)s"
)


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


class TestPIDSimulatedTime(unittest.TestCase):

    def setUp(self):
        logging.getLogger().setLevel(logging.CRITICAL)
        self.simulated_time = SimulatedTime()
        self.pid = PID(
            kp=1.0, ki=0.1, kd=0.01, timeFunc=self.simulated_time.get_time, intMax=100.0, intMin=-100.0
        )

    def tearDown(self):
        self.pid.clearIntResult()

    def test_initialization(self):
        self.assertEqual(self.pid.kp, 1.0)
        self.assertEqual(self.pid.ki, 0.1)
        self.assertEqual(self.pid.kd, 0.01)
        self.assertEqual(self.pid._PID__intMax, 100.0)
        self.assertEqual(self.pid._PID__intMin, -100.0)

    def test_pid_computation(self):
        result = self.pid.compute(1000.0)
        self.assertIsInstance(result, float)

    def test_integral_clamping(self):
        for _ in range(10):
            self.simulated_time.advance_time(0.1)
            self.pid.compute(1000.0)
        self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)

    def test_derivative_computation(self):
        result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
        self.assertIsInstance(result, float)

    def test_reset_integral(self):
        self.simulated_time.advance_time(1.0)
        self.pid.compute(500.0)
        self.pid.clearIntResult()
        self.assertEqual(self.pid._PID__errIntegral, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)