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


if __name__ == "__main__":
    unittest.main(verbosity=2)