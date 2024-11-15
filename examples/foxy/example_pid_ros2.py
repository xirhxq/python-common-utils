# examples/foxy/example_pid_ros2.py

import rclpy
from rclpy.node import Node
from core.PID import PID


class ExamplePIDROS2(Node):
    def __init__(self):
        super().__init__("example_pid_node")

        self.pid = PID(
            kp=1.0,
            ki=0.1,
            kd=0.05,
            timeFunc=lambda: self.get_clock().now().nanoseconds * 1e-9,
            intMax=10.0,
            intMin=-10.0,
        )

        self.error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]
        self.pid_outputs = []

        self.run_pid_example()

    def run_pid_example(self):
        for error in self.error_values:
            output = self.pid.compute(error)
            self.pid_outputs.append(output)
            self.get_logger().info(f"Error: {error}, PID Output: {output}")

        self.pid.clearIntResult()
        self.get_logger().info(
            f"Integral result after clearing: {self.pid._PID__errIntegral}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePIDROS2()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
