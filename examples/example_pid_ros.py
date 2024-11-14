# examples/example_pid_ros.py

import rospy
from core.PID import PID


def example_pid_ros():
    rospy.init_node("example_pid_node", anonymous=True)

    pid = PID(
        kp=1.0, ki=0.1, kd=0.05, timeFunc=rospy.get_time, intMax=10.0, intMin=-10.0
    )

    error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]

    pid_outputs = []

    for error in error_values:
        output = pid.compute(error)
        pid_outputs.append(output)
        rospy.loginfo(f"Error: {error}, PID Output: {output}")

    pid.clearIntResult()
    rospy.loginfo(f"Integral result after clearing: {pid._PID__errIntegral}")


if __name__ == "__main__":
    example_pid_ros()
