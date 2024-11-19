# tests/noetic/test_time_buffer_ros.py

import subprocess
import time
import rospy
from tests.common.test_time_buffer_base import TestTimeBufferBase


class TestTimeBufferROS(TestTimeBufferBase.TestTimeBufferBase):
    @classmethod
    def setUpClass(cls):
        cls.roscore_process = subprocess.Popen(
            ["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        time.sleep(5)

        if not rospy.core.is_initialized():
            try:
                rospy.init_node(
                    "time_buffer_test_ros", anonymous=True, log_level=rospy.FATAL
                )
            except rospy.exceptions.ROSInitException as e:
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

    def get_time_func(self):
        return rospy.get_time

    def reset_time(self):
        pass

    def advance_time(self, delta):
        rospy.sleep(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
