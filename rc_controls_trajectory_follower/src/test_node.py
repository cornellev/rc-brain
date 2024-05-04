#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg, TrajectoryPointMsg
import math


if __name__ == "__main__":
    rospy.init_node("test_node")

    dT = 1 / 6

    state_steps = []

    # for i in range(30):
    #     msg = TrajectoryPointMsg()
    #     msg.speed = 1.0
    #     msg.steering_angle = math.cos(6 * i * dT + (math.pi / 2)) * 20
    #     state_steps.append(msg)

    for i in range(30):
        msg = TrajectoryPointMsg()
        msg.speed = 1.5
        msg.steering_angle = 0
        state_steps.append(msg)

    msg = TrajectoryMsg()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.dt = dT
    msg.trajectory = state_steps

    rate = rospy.Rate(1)
    pub = rospy.Publisher("/trajectory_msg", TrajectoryMsg, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("Trajectory published")
