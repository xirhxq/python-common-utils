# tests/common/test_pid_base.py

import unittest
from core.PID import PID


class TestPIDBase:

    class TestPIDBase(unittest.TestCase):
        def setUp(self):
            self.time_func = self.get_time_func()
            self.pid = PID(
                kp=1.0,
                ki=0.1,
                kd=0.01,
                timeFunc=self.time_func,
                intMax=100.0,
                intMin=-100.0,
            )
            self.reset_time()

        def tearDown(self):
            self.pid.clearIntResult()

        def get_time_func(self):
            raise NotImplementedError

        def reset_time(self):
            raise NotImplementedError

        def advance_time(self, delta):
            raise NotImplementedError

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
                self.advance_time(0.1)
                self.pid.compute(1000.0)
            self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)

        def test_derivative_computation(self):
            result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
            self.assertIsInstance(result, float)

        def test_reset_integral(self):
            self.advance_time(1.0)
            self.pid.compute(500.0)
            self.pid.clearIntResult()
            self.assertEqual(self.pid._PID__errIntegral, 0.0)
