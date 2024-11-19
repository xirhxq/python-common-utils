# tests/noetic/test_time_buffer_system_time.py

import time
from tests.common.test_time_buffer_base import TestTimeBufferBase


class TestTimeBufferSystemTime(TestTimeBufferBase.TestTimeBufferBase):
    def get_time_func(self):
        return time.time

    def reset_time(self):
        pass

    def advance_time(self, delta):
        time.sleep(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
