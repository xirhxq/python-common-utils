# examples/noetic/example_average_filter_ros.py

import rospy
from core.AverageFilter import AverageFilter


def average_filter_ros():
    avg_filter = AverageFilter(max_age=0.15, time_func=rospy.get_time)
    start_time = rospy.get_time()
    max_duration = 1
    value = 10.0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if current_time - start_time > max_duration:
            rospy.loginfo("Reached maximum duration, stopping the node.")
            break

        avg_filter.add_value(value)
        avg = avg_filter.get_average()
        rospy.loginfo(
            "Current average: {}".format(avg if avg is not None else "N/A")
        )
        value += 5.0  # Increment value for demonstration
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("average_filter_ros_node")
    average_filter_ros()