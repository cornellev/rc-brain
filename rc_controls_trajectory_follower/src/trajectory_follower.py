#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
import numpy as np
import numpy.typing as npt
from typing import Union

ANGLE_KP = 0
ANGLE_KI = 0
ANGLE_KD = 0
ANGLE_KF = 1
ANGLE_I_MAX = 10


def vec3_to_np(vec3) -> npt.NDArray:
    return np.array([vec3.x, vec3.y, vec3.z])


def interpolate(lo: float, hi: float, percent: float) -> float:
    return percent * hi + (1 - percent) * lo


class PID:
    kP: float
    kI: float
    kD: float
    i_min: float
    i_max: float

    last_time: Union[rospy.Time, None]
    last_error: Union[float, None]
    error_integral: float

    def __init__(self, kP, kI, kD, i_bounds) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.i_min, self.i_max = i_bounds

        self.last_time = None
        self.last_error = None
        self.error_integral = 0

    def calculate(self, error: float) -> float:
        if self.last_time is None:
            self.last_time = rospy.Time.now()
            self.last_error = error
            return 0

        now = rospy.Time.now()
        dt = now - self.last_time

        if self.last_error is None:
            self.last_error = error

        result = (
            self.kP * error
            + self.kI * self.error_integral
            + self.kD * (error - self.last_error) / dt
        )

        self.last_time = now
        self.last_error = error
        self.error_integral += error * dt
        self.error_integral = np.clip(self.error_integral, self.i_min, self.i_max)

        return result


class TrajectoryFollower:
    """
    Class for the trajectory follower node. This node subscribes to the /trajectory_msg topic and publishes AckermannDrive messages to the /rc_movement_msg topic.
    """

    trajectory: Union[TrajectoryMsg, None]
    odom: Union[Odometry, None]
    pid: PID

    def __init__(self):
        self.trajectory_sub = rospy.Subscriber(
            "/trajectory_msg", TrajectoryMsg, self.trajectory_callback
        )
        self.odom_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odometry_callback
        )
        self.pub = rospy.Publisher("/rc_movement_msg", AckermannDrive, queue_size=1)
        self.trajectory = None
        self.pose = None
        self.pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, (-ANGLE_I_MAX, ANGLE_I_MAX))

        rospy.loginfo("Trajectory follower node initialized.")

    def trajectory_callback(self, msg):
        """
        Callback for the trajectory message. Simply stores the last received trajectory message.

        Args:
            msg (TrajectoryMsg): The received trajectory message.
        """

        rospy.loginfo("Received new trajectory message.")

        self.trajectory = msg

    def odometry_callback(self, msg: Odometry):
        self.odom = msg

    def cross_track_error(
        self, path0: npt.NDArray, path1: npt.NDArray, pos: npt.NDArray
    ) -> float:
        """
        See notion for derivation (I will put this there eventually)
        """
        traj_vec = path1 - path0
        relative_pos = pos - path0
        det = np.linalg.det(np.column_stack((relative_pos, traj_vec)))
        return det / np.linalg.norm(traj_vec)

    def calculate_setpoint(self) -> AckermannDrive:
        """
        Calculates the current velocity/heading setpoint based on the current time and the last received trajectory message.

        Returns:
            AckermannDrive: The current velocity/heading setpoint.
        """
        assert self.ready()

        # TODO: project time forward
        now = rospy.Time.now()
        segment_start_index = int(
            (now - self.trajectory.header.stamp) / self.trajectory.dt
        )
        segment_end_index = segment_start_index + 1
        num_points = len(self.trajectory.trajectory)

        if segment_end_index >= num_points:
            result = AckermannDrive()
            result.speed = 0
            result.steering_angle = self.trajectory.trajectory[-1].steering_angle
            return result

        start = self.trajectory.trajectory[segment_start_index]
        end = self.trajectory.trajectory[segment_end_index]

        # TODO: project position forward
        position = vec3_to_np(self.odom.pose.pose.position)
        path_point_0 = vec3_to_np(start.position)
        path_point_1 = vec3_to_np(end.position)

        # TODO: should we calculate cross track error to closest trajectory segment instead?
        xte = self.cross_track_error(path_point_0, path_point_1, position)

        result = AckermannDrive()

        segment_start_time = (
            self.trajectory.header.stamp + segment_start_index * self.trajectory.dt
        )
        percent_along = (now - segment_start_time) / self.trajectory.dt
        result.speed = interpolate(start.speed, end.speed, percent_along)

        result.steering_angle = self.pid.calculate(xte) + ANGLE_KF * interpolate(
            start.steering_angle, end.steering_angle, percent_along
        )

    def ready(self) -> bool:
        return self.trajectory is not None and self.odom is not None


if __name__ == "__main__":
    rospy.init_node("rc_controls_trajectory_follower")
    follower = TrajectoryFollower()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if follower.ready():
            setpoint = follower.calculate_setpoint()
            follower.pub.publish(setpoint)
        rate.sleep()
