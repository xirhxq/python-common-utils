# tests/noetic/test_pid_ros.py

import rospy
import subprocess
import time
from tests.common.test_pid_base import TestPIDBase


class TestPIDROS(TestPIDBase.TestPIDBase):
    @classmethod
    def setUpClass(cls):
        cls.roscore_process = subprocess.Popen(
            ["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        time.sleep(5)
        if not rospy.core.is_initialized():
            try:
                rospy.init_node("pid_test_ros", anonymous=True, log_level=rospy.FATAL)
            except rospy.exceptions.ROSInitException:
                raise

    @classmethod
    def tearDownClass(cls):
        rospy.signal_shutdown("Shutting down test node")
        cls.roscore_process.terminate()
        cls.roscore_process.wait()

    def get_time_func(self):
        return rospy.get_time

    def reset_time(self):
        pass

    def advance_time(self, delta):
        rospy.sleep(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
