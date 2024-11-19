# tests/foxy/test_time_buffer_ros2.py

import time
import rclpy
from rclpy.node import Node
from tests.common.test_time_buffer_base import TestTimeBufferBase


class TestTimeBufferROS2(TestTimeBufferBase.TestTimeBufferBase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("time_buffer_test_ros2")
        rclpy.logging.set_logger_level(
            "time_buffer_test_ros2", rclpy.logging.LoggingSeverity.FATAL
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def get_time_func(self):
        return lambda: self.node.get_clock().now().nanoseconds * 1e-9

    def reset_time(self):
        pass

    def advance_time(self, delta):
        time.sleep(delta)


if __name__ == "__main__":
    unittest.main(verbosity=2)
