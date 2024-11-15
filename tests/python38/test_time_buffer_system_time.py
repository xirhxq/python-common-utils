# tests/python38/test_time_buffer_system_time.py

import unittest
import time
import sys
import os
from core.TimeBuffer import TimeBuffer


class TestTimeBufferSystemTime(unittest.TestCase):

    def setUp(self):
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
        self.buffer = TimeBuffer(name="TestBuffer", max_age=0.15, time_func=time.time)

    def tearDown(self):
        sys.stdout.close()
        sys.stderr.close()
        sys.stdout = self._original_stdout
        sys.stderr = self._original_stderr

    def test_add_and_retrieve_message(self):
        self.buffer.add_message("TestMessage")
        result = self.buffer.get_message()
        self.assertIsNone(
            result, "Should return None if no message is older than max_age"
        )

        time.sleep(0.2)
        result = self.buffer.get_message()
        self.assertEqual(
            result,
            "TestMessage",
            "Should retrieve the correct message after max_age has passed",
        )

    def test_message_expiry(self):
        self.buffer.add_message("OldMessage")
        time.sleep(0.1)
        self.buffer.add_message("NewMessage")
        time.sleep(0.1)
        result = self.buffer.get_message()
        self.assertEqual(
            result, "OldMessage", "Should return the last message before max_age limit"
        )

    def test_buffer_empty_property(self):
        self.assertTrue(self.buffer.is_empty, "Buffer should be empty initially")
        self.buffer.add_message("NewMessage")
        self.assertFalse(
            self.buffer.is_empty, "Buffer should not be empty after adding a message"
        )
        time.sleep(0.2)
        self.buffer.get_message()
        self.assertTrue(
            self.buffer.is_empty, "Buffer should be empty after message expires"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
