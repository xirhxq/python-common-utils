# tests/python38/test_time_buffer_simulated_time.py

import unittest
from core.TimeBuffer import TimeBuffer


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


class TestTimeBufferSimulatedTime(unittest.TestCase):

    def setUp(self):
        self.simulated_time = SimulatedTime()
        self.buffer = TimeBuffer(
            name="TestBuffer", max_age=0.15, time_func=self.simulated_time.get_time
        )

    def test_add_and_retrieve_message(self):
        self.buffer.add_message("TestMessage")
        result = self.buffer.get_message()
        self.assertIsNone(
            result, "Should return None if no message is older than max_age"
        )

        self.simulated_time.advance_time(0.2)
        result = self.buffer.get_message()
        self.assertEqual(
            result,
            "TestMessage",
            "Should retrieve the correct message after max_age has passed",
        )

    def test_message_expiry(self):
        self.buffer.add_message("OldMessage")
        self.simulated_time.advance_time(0.1)
        self.buffer.add_message("NewMessage")
        self.simulated_time.advance_time(0.1)
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
        self.simulated_time.advance_time(0.2)
        self.buffer.get_message()
        self.assertTrue(
            self.buffer.is_empty, "Buffer should be empty after message expires"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)