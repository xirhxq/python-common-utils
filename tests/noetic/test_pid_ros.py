# tests/noetic/test_pid_ros.py

import time
import unittest
import subprocess
import rospy
import sys
import os
from core.PID import PID


class TestPIDROS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.roscore_process = subprocess.Popen(
            ["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        time.sleep(5)

        if not rospy.core.is_initialized():
            try:
                rospy.init_node("pid_test_ros", anonymous=True, log_level=rospy.FATAL)
            except rospy.exceptions.ROSInitException as e:
                rospy.logerr(f"Failed to initialize ROS node: {e}")
                raise

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown("Shutting down test node")
        cls.roscore_process.terminate()
        cls.roscore_process.wait()
        if cls.roscore_process.stdout:
            cls.roscore_process.stdout.close()
        if cls.roscore_process.stderr:
            cls.roscore_process.stderr.close()

    def setUp(self):
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        sys.stdout = open(os.devnull, "w")
        sys.stderr = open(os.devnull, "w")
        self.pid = PID(
            kp=1.0,
            ki=0.1,
            kd=0.01,
            timeFunc=rospy.get_time,
            intMax=100.0,
            intMin=-100.0,
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
            rospy.sleep(0.1)
        self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)

    def test_derivative_computation(self):
        result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
        self.assertIsInstance(result, float)


if __name__ == "__main__":
    unittest.main(verbosity=2)
