#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg, TrajectoryPointMsg
from ackermann_msgs.msg import AckermannDrive


def gen_line():
    p0 = TrajectoryPointMsg()
    p0.speed = 0
    p0.steering_angle = 0

    p1 = TrajectoryPointMsg()
    p1.speed = 1
    p1.steering_angle = 0

    p2 = TrajectoryPointMsg()
    p2.speed = 0
    p2.steering_angle = 0

    msg = TrajectoryMsg()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.dt = 1
    msg.trajectory = [p0, p1, p2]


if __name__ == "__main__":
    rospy.init_node("test_node")

    p0 = TrajectoryPointMsg()
    p0.speed = 0.5
    p0.steering_angle = 1

    p1 = TrajectoryPointMsg()
    p1.speed = 0.8
    p1.steering_angle = 0.5

    msg = TrajectoryMsg()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.dt = 0.5
    msg.trajectory = [p0, p1]

    rate = rospy.Rate(10)
    pub = rospy.Publisher("/trajectory_msg", TrajectoryMsg, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
