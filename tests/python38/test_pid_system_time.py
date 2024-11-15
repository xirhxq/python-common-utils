# tests/python38/test_pid_system_time.py

import time
import unittest
import logging
import sys
import os
from core.PID import PID

logging.basicConfig(
    level=logging.CRITICAL, format="%(asctime)s - %(levelname)s - %(message)s"
)


class TestPIDSystemTime(unittest.TestCase):

    def setUp(self):
        logging.getLogger().setLevel(logging.CRITICAL)
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
        self.pid = PID(
            kp=1.0, ki=0.1, kd=0.01, timeFunc=time.time, intMax=100.0, intMin=-100.0
        )

    def tearDown(self):
        sys.stdout.close()
        sys.stderr.close()
        sys.stdout = self._original_stdout
        sys.stderr = self._original_stderr
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
            self.pid.compute(1000.0)
            time.sleep(0.1)
        self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)

    def test_derivative_computation(self):
        result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
        self.assertIsInstance(result, float)


if __name__ == "__main__":
    unittest.main(verbosity=2)
