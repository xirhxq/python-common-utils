# tests/python38/test_pid_system_time.py

import time
from tests.common.test_pid_base import TestPIDBase


class TestPIDSystemTime(TestPIDBase.TestPIDBase):
    def get_time_func(self):
        return time.time

    def reset_time(self):
        pass

    def advance_time(self, delta):
        time.sleep(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
