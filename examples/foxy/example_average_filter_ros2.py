# examples/foxy/example_average_filter_ros2.py

import rclpy
from rclpy.node import Node
from core.AverageFilter import AverageFilter
import time


class AverageFilterROS2(Node):
    def __init__(self):
        super().__init__("average_filter_ros2_node")
        self.avg_filter = AverageFilter(
            max_age=0.15, time_func=lambda: self.get_clock().now().nanoseconds * 1e-9
        )
        self.value = 10.0

    def run_example(self, max_duration=1.0):
        start_time = self.get_clock().now().nanoseconds * 1e-9
        while (self.get_clock().now().nanoseconds * 1e-9 - start_time) < max_duration:
            self.avg_filter.add_value(self.value)
            avg = self.avg_filter.get_average()
            self.get_logger().info(
                f"Current average: {avg if avg is not None else 'N/A'}"
            )
            self.value += 5.0  # Increment value for demonstration
            time.sleep(0.1)

        self.get_logger().info("Reached maximum duration, stopping the node.")


def main(args=None):
    rclpy.init(args=args)
    node = AverageFilterROS2()
    node.run_example(max_duration=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()