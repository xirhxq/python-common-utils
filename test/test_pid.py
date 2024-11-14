# test/test_pid.py

import time
import unittest
import logging
import subprocess
import rospy
from core.PID import PID

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class TestPIDSystemTime(unittest.TestCase):
    
    def setUp(self):
        self.pid = PID(kp=1.0, ki=0.1, kd=0.01, timeFunc=time.time, intMax=100.0, intMin=-100.0)
        logging.info("Setup: PID controller initialized with kp=1.0, ki=0.1, kd=0.01, intMax=100.0, intMin=-100.0")
    
    def tearDown(self):
        logging.info("Tear down: Resetting PID controller")
        self.pid.clearIntResult()

    def test_initialization(self):
        self.assertEqual(self.pid.kp, 1.0)
        self.assertEqual(self.pid.ki, 0.1)
        self.assertEqual(self.pid.kd, 0.01)
        self.assertEqual(self.pid._PID__intMax, 100.0)
        self.assertEqual(self.pid._PID__intMin, -100.0)
        logging.debug(f"PID controller initialized: kp={self.pid.kp}, ki={self.pid.ki}, kd={self.pid.kd}, intMax={self.pid._PID__intMax}, intMin={self.pid._PID__intMin}")
    
    def test_pid_computation(self):
        result = self.pid.compute(1000.0)
        logging.debug(f"PID output for error 1000.0: {result}")
        self.assertIsInstance(result, float)
    
    def test_integral_clamping(self):
        for _ in range(10):
            self.pid.compute(1000.0)
            time.sleep(0.1)
        self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)
        logging.debug(f"Integral term clamped to: {self.pid._PID__errIntegral}")

    def test_derivative_computation(self):
        result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
        logging.debug(f"PID derivative term with physical diff: {result}")
        self.assertIsInstance(result, float)

class TestPIDROS(unittest.TestCase):

    def setUp(self):
        self.roscore_process = subprocess.Popen(['roscore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)

        try:
            rospy.init_node('pid_test_ros', anonymous=True)
        except rospy.exceptions.ROSInitException as e:
            logging.error(f"Failed to initialize ROS node: {e}")
            raise

        self.pid = PID(kp=1.0, ki=0.1, kd=0.01, timeFunc=rospy.get_time, intMax=100.0, intMin=-100.0)
        logging.info("Setup: PID controller initialized with kp=1.0, ki=0.1, kd=0.01, intMax=100.0, intMin=-100.0")
    
    def tearDown(self):
        self.roscore_process.terminate()
        self.roscore_process.wait()
        if self.roscore_process.stdout:
            self.roscore_process.stdout.close()
        if self.roscore_process.stderr:
            self.roscore_process.stderr.close()

        logging.info("Tear down: Stopped roscore process and resetting PID controller")
        self.pid.clearIntResult()

    def test_initialization(self):
        self.assertEqual(self.pid.kp, 1.0)
        self.assertEqual(self.pid.ki, 0.1)
        self.assertEqual(self.pid.kd, 0.01)
        self.assertEqual(self.pid._PID__intMax, 100.0)
        self.assertEqual(self.pid._PID__intMin, -100.0)
        logging.debug(f"PID controller initialized: kp={self.pid.kp}, ki={self.pid.ki}, kd={self.pid.kd}, intMax={self.pid._PID__intMax}, intMin={self.pid._PID__intMin}")
    
    def test_pid_computation(self):
        result = self.pid.compute(1000.0)
        logging.debug(f"PID output for error 1000.0: {result}")
        self.assertIsInstance(result, float)
    
    def test_integral_clamping(self):
        for _ in range(10):
            self.pid.compute(1000.0)
            rospy.sleep(0.1)
        self.assertEqual(self.pid._PID__errIntegral, self.pid._PID__intMax)
        logging.debug(f"Integral term clamped to: {self.pid._PID__errIntegral}")

    def test_derivative_computation(self):
        result = self.pid.compute(1000.0, errPhysicalDiff=2.0)
        logging.debug(f"PID derivative term with physical diff: {result}")
        self.assertIsInstance(result, float)

if __name__ == "__main__":
    unittest.main(verbosity=2)
