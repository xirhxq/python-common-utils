# tests/foxy/test_average_filter_ros2.py

import time
import rclpy
from rclpy.node import Node
from tests.common.test_average_filter_base import TestAverageFilterBase


class TestAverageFilterROS2(TestAverageFilterBase.TestAverageFilterBase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("average_filter_test_ros2")
        rclpy.logging.set_logger_level(
            "average_filter_test_ros2", rclpy.logging.LoggingSeverity.FATAL
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