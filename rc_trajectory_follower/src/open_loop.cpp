#include <ros/ros.h>
#include <rc_trajectory_follower/TrajectoryMsg.h>
#include <ackermann_msgs/AckermannDrive.h>

rc_trajectory_follower::TrajectoryMsg last_trajectory;

ackermann_msgs::AckermannDrive calculate_setpoint(rc_trajectory_follower::TrajectoryMsg trajectory)
{
    int index = (ros::Time::now() - trajectory.header.stamp).toSec() / trajectory.dt;

    ackermann_msgs::AckermannDrive current;
    current.speed = trajectory.trajectory[index].speed;
    current.steering_angle = trajectory.trajectory[index].steering_angle;

    return current;
}

void trajectory_callback(rc_trajectory_follower::TrajectoryMsg trajectory)
{
    last_trajectory = trajectory;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_loop_node");
    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("trajectory_msg", 10, trajectory_callback);
    ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDrive>("rc_movement_msg", 10);

    ros::Rate rate = ros::Rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}