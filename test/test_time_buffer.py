# test/test_time_buffer.py

import unittest
import subprocess
import time
import logging
import rospy
from core.TimeBuffer import TimeBuffer

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class TestTimeBuffer(unittest.TestCase):

    def setUp(self):
        self.buffer = TimeBuffer(name='TestBuffer', max_age=0.15, time_func=time.time)

    def test_add_and_retrieve_message(self):
        self.buffer.add_message("TestMessage")
        result = self.buffer.get_message()
        self.assertIsNone(result, "Should return None if no message is older than max_age")

        time.sleep(0.2)
        result = self.buffer.get_message()
        self.assertEqual(result, "TestMessage", "Should retrieve the correct message after max_age has passed")

    def test_message_expiry(self):
        self.buffer.add_message("OldMessage")
        time.sleep(0.1)
        self.buffer.add_message("NewMessage")
        time.sleep(0.1)
        result = self.buffer.get_message()
        self.assertEqual(result, "OldMessage", "Should return the last message before max_age limit")

    def test_buffer_empty_property(self):
        self.assertTrue(self.buffer.is_empty, "Buffer should be empty initially")
        self.buffer.add_message("NewMessage")
        self.assertFalse(self.buffer.is_empty, "Buffer should not be empty after adding a message")
        time.sleep(0.2)
        self.buffer.get_message()
        self.assertTrue(self.buffer.is_empty, "Buffer should be empty after message expires")


class TestTimeBufferROS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.roscore_process = subprocess.Popen(['roscore'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(5)

        if not rospy.core.is_initialized():
            try:
                rospy.init_node('time_buffer_test_ros', anonymous=True)
            except rospy.exceptions.ROSInitException as e:
                logging.error(f"Failed to initialize ROS node: {e}")
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
        # 使用 ROS 时间初始化 TimeBuffer
        self.buffer = TimeBuffer(name='TestBufferROS', max_age=0.15, time_func=rospy.get_time)
        logging.info("Setup: TimeBuffer initialized with ROS time")

    def test_add_and_retrieve_message(self):
        self.buffer.add_message("TestMessage")
        result = self.buffer.get_message()
        self.assertIsNone(result, "Should return None if no message is older than max_age")

        rospy.sleep(0.2)
        result = self.buffer.get_message()
        self.assertEqual(result, "TestMessage", "Should retrieve the correct message after max_age has passed")

    def test_message_expiry(self):
        self.buffer.add_message("OldMessage")
        rospy.sleep(0.1)
        self.buffer.add_message("NewMessage")
        rospy.sleep(0.1)
        result = self.buffer.get_message()
        self.assertEqual(result, "OldMessage", "Should return the last message before max_age limit")

    def test_buffer_empty_property(self):
        self.assertTrue(self.buffer.is_empty, "Buffer should be empty initially")
        self.buffer.add_message("NewMessage")
        self.assertFalse(self.buffer.is_empty, "Buffer should not be empty after adding a message")
        rospy.sleep(0.2)
        self.buffer.get_message()  # Trigger message cleanup
        self.assertTrue(self.buffer.is_empty, "Buffer should be empty after message expires")

if __name__ == "__main__":
    unittest.main(verbosity=2)