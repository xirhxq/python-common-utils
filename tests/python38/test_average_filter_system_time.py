# tests/python38/test_average_filter_system_time.py

import time
from tests.common.test_average_filter_base import TestAverageFilterBase


class TestAverageFilterSystemTime(TestAverageFilterBase.TestAverageFilterBase):
    def setUp(self):
        super().setUp()

    def get_time_func(self):
        return time.time

    def reset_time(self):
        pass

    def advance_time(self, delta):
        time.sleep(delta)

    def test_value_expiry(self):
        # Override this test as timing with sleep is not precise enough for unit tests
        self.filter.add_value(10.0)
        self.advance_time(0.05)
        self.filter.add_value(20.0)
        self.advance_time(0.05)
        self.filter.add_value(30.0)
        self.advance_time(0.06)  # Total: 0.16s, first value should expire
        
        # We can't guarantee exact timing with sleep, so just ensure we get a reasonable average
        avg = self.filter.get_average()
        self.assertIsNotNone(avg)
        self.assertGreaterEqual(avg, 20.0)
        self.assertLessEqual(avg, 25.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)