#!/usr/bin/env python3

import rospy
from rc_controls_trajectory_follower.msg import TrajectoryMsg
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
import numpy as np
import numpy.typing as npt
from typing import Union


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
        self.odom = None
        self.pose = None
        self.pid = PID(angle_kp, angle_ki, angle_kd, (-angle_i_max, angle_i_max))

    def trajectory_callback(self, msg: TrajectoryMsg):
        """
        Callback for the trajectory message. Simply stores the last received trajectory message.

        Args:
            msg (TrajectoryMsg): The received trajectory message.
        """

        self.trajectory = msg
        if msg.header.frame_id != odom_frame:
            rospy.logwarn(
                "Trajectory is expected to be in the odom frame, but it is not. \
                The trajectory follower may behave unpredictably."
            )

    def odometry_callback(self, msg: Odometry):
        self.odom = msg
        if msg.header.frame_id != odom_frame:
            rospy.logwarn(
                "Odometry is expected to be in the odom frame, but it is not. \
                The trajectory follower may behave unpredictably."
            )

        if msg.child_frame_id != base_link_frame:
            rospy.logwarn(
                "Odometry message is expected to provide the pose of base_link_frame \
                relative to odom_frame, but it does not. child_frame_id should be the \
                same frame as base_link_frame. The trajectory follower may behave unpredictably."
            )

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

        result.steering_angle = self.pid.calculate(xte) + angle_kf * interpolate(
            start.steering_angle, end.steering_angle, percent_along
        )

        return result

    def ready(self) -> bool:
        return self.trajectory is not None and self.odom is not None


if __name__ == "__main__":
    rospy.init_node("rc_controls_trajectory_follower")

    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_link_frame = rospy.get_param("~base_link_frame", "base_link")

    steering_config = rospy.get_param("~steering")
    angle_kp = steering_config["kp"]
    angle_ki = steering_config["ki"]
    angle_kd = steering_config["kd"]
    angle_kf = steering_config["kf"]
    angle_i_max = steering_config["i_max"]

    rospy.loginfo("odom frame = %s, base link frame = %s", odom_frame, base_link_frame)
    rospy.loginfo(
        "Steering control parameters: kp=%.3f, ki=%.3f, kd=%.3f, kf=%.3f, i_max=%.3f",
        angle_kp,
        angle_ki,
        angle_kd,
        angle_kf,
        angle_i_max,
    )

    follower = TrajectoryFollower()
    rospy.loginfo("Trajectory follower node initialized.")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if follower.ready():
            setpoint = follower.calculate_setpoint()
            follower.pub.publish(setpoint)
        rate.sleep()
