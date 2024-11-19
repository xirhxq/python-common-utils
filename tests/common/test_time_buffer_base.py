# tests/common/time_buffer_test_base.py

import unittest
from core.TimeBuffer import TimeBuffer


class TestTimeBufferBase:

    class TestTimeBufferBase(unittest.TestCase):
        TIME_ADVANCE_STEP = 0.1

        def setUp(self):
            self.time_func = self.get_time_func()
            self.buffer = TimeBuffer(
                name="TestBuffer", max_age=0.15, time_func=self.time_func
            )
            self.reset_time()

        def get_time_func(self):
            raise NotImplementedError

        def reset_time(self):
            raise NotImplementedError

        def advance_time(self, delta):
            raise NotImplementedError

        def test_add_and_retrieve_message(self):
            self.buffer.add_message("TestMessage")
            result = self.buffer.get_message()
            self.assertIsNone(
                result, "Should return None if no message is older than max_age"
            )

            self.advance_time(0.2)
            result = self.buffer.get_message()
            self.assertEqual(
                result,
                "TestMessage",
                "Should retrieve the correct message after max_age has passed",
            )

        def test_message_expiry(self):
            self.buffer.add_message("OldMessage")
            self.advance_time(0.1)
            self.buffer.add_message("NewMessage")
            self.advance_time(0.1)
            result = self.buffer.get_message()
            self.assertEqual(
                result,
                "OldMessage",
                "Should return the last message before max_age limit",
            )

        def test_buffer_empty_property(self):
            self.assertTrue(self.buffer.is_empty, "Buffer should be empty initially")
            self.buffer.add_message("NewMessage")
            self.assertFalse(
                self.buffer.is_empty,
                "Buffer should not be empty after adding a message",
            )
            self.advance_time(0.2)
            self.buffer.get_message()
            self.assertTrue(
                self.buffer.is_empty, "Buffer should be empty after message expires"
            )
