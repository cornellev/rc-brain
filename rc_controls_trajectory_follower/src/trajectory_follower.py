#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg
from ackermann_msgs.msg import AckermannDrive

class TrajectoryFollower:
    def __init__(self):
        self.sub = rospy.Subscriber("/trajectory_msg", TrajectoryMsg, self.trajectory_callback)
        self.pub = rospy.Publisher("/rc_movement_msg", AckermannDrive, queue_size=1)
        self.last_trajectory = None
        self.ready = False

        rospy.loginfo("Trajectory follower node initialized.")

    def trajectory_callback(self, msg):
        self.last_trajectory = msg
        self.ready = True

    def calculate_setpoint(self):
        index = int((rospy.Time.now() - self.last_trajectory.header.stamp).to_sec() / self.last_trajectory.dt)
        num_setpoints = len(self.last_trajectory.speeds)

        current = AckermannDrive()

        if index >= num_setpoints:
            current.speed = 0
            current.steering_angle = self.last_trajectory.steering_angles[-1]
            
            return current

        current.speed = self.last_trajectory.speeds[index]
        current.steering_angle = self.last_trajectory.steering_angles[index]

        return current


if __name__ == "__main__": 
    rospy.init_node("rc_controls_trajectory_follower")
    follower = TrajectoryFollower()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if follower.ready:
            setpoint = follower.calculate_setpoint()
            follower.pub.publish(setpoint)
        rate.sleep()
