# examples/foxy/example_time_buffer_ros2.py

import rclpy
from rclpy.node import Node
from core.TimeBuffer import TimeBuffer
import time


class TimeBufferROS2(Node):
    def __init__(self):
        super().__init__("timebuffer_ros2_node")
        self.time_buffer = TimeBuffer(
            max_age=0.15, time_func=lambda: self.get_clock().now().nanoseconds * 1e-9
        )

    def run_example(self, max_duration=1.0):
        start_time = self.get_clock().now().nanoseconds * 1e-9
        while (self.get_clock().now().nanoseconds * 1e-9 - start_time) < max_duration:
            self.time_buffer.add_message("ROS 2 message")
            last_message = self.time_buffer.get_message()
            self.get_logger().info(
                f"Last valid message before time limit: {last_message}"
            )
            time.sleep(0.1)

        self.get_logger().info("Reached maximum duration, stopping the node.")


def main(args=None):
    rclpy.init(args=args)
    node = TimeBufferROS2()
    node.run_example(max_duration=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
