#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg, TrajectoryPointMsg
import math


if __name__ == "__main__":
    rospy.init_node("test_node")

    dT = 1/6

    state_steps = []

    # for i in range(30):
    #     state_steps.append(TrajectoryPointMsg())
    #     state_steps[-1].speed = .7
    #     state_steps[-1].steering_angle = math.cos(6 * i * dT + (math.pi/2))

    for i in range(30):
        state_steps.append(TrajectoryPointMsg())
        state_steps[-1].speed = 1.5
        state_steps[-1].steering_angle = 0

    msg = TrajectoryMsg()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.dt = dT
    msg.trajectory = state_steps

    rate = rospy.Rate(1)
    pub = rospy.Publisher("/trajectory_msg", TrajectoryMsg, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("Trajectory published")
