# examples/noetic/example_time_buffer_ros.py

import rospy
from core.TimeBuffer import TimeBuffer


def timebuffer_ros():
    time_buffer = TimeBuffer(max_age=0.15, time_func=rospy.get_time)
    start_time = rospy.get_time()
    max_duration = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        if current_time - start_time > max_duration:
            rospy.loginfo("Reached maximum duration, stopping the node.")
            break

        time_buffer.add_message("ROS message")
        rospy.loginfo(
            "Last valid message before time limit: {}".format(time_buffer.get_message())
        )
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("timebuffer_ros_node")
    timebuffer_ros()
