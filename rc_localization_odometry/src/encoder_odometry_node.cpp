#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rc_localization_odometry/SensorCollect.h>

const double BIKE_LENGTH = 19.0 / 100;
const double WHEEL_DIAMETER_M = 9.5 / 100;
const double TICKS_PER_REV = 827.2;
const double TICKS_TO_M = (1 / TICKS_PER_REV) * (2 * M_PI * (WHEEL_DIAMETER_M / 2));

std::string frame;
std::string sensor_topic;
std::string odom_topic;
int pub_rate;
std::vector<float> pose_var;
std::vector<float> velo_var;

bool first = true;
rc_localization_odometry::SensorCollect last;

double steer_angle = 0.0;
double x, y, theta, x_dot, y_dot, theta_dot = 0;

void data_callback(rc_localization_odometry::SensorCollect current)
{
    if (first)
    {
        first = false;
        last = current;
        return;
    }

    ros::Duration dt = ros::Duration((current.timestamp - last.timestamp) / 1000.0);

    double delta_enc_m = current.velocity;

    last = current;

    double v = delta_enc_m / dt.toSec();
    double steer_angle = current.steering_angle;

    // should be new theta?
    x_dot = v * std::cos(theta);
    y_dot = v * std::sin(theta);
    theta_dot = v * std::tan(steer_angle) / BIKE_LENGTH;

    x += x_dot * dt.toSec();
    y += y_dot * dt.toSec();
    theta += theta_dot * dt.toSec();
}

nav_msgs::Odometry build_odom_packet()
{
    nav_msgs::Odometry odom;

    odom.header.frame_id = frame;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.covariance[0] = pose_var[0];
    odom.pose.covariance[7] = pose_var[1];
    odom.pose.covariance[35] = pose_var[2];

    tf2::Quaternion q;
    q.setEuler(theta, 0, 0);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = x_dot;
    odom.twist.twist.linear.y = y_dot;
    odom.twist.twist.linear.z = 0;
    odom.twist.covariance[0] = velo_var[0];
    odom.twist.covariance[7] = velo_var[1];
    odom.twist.covariance[35] = velo_var[2];

    odom.twist.twist.angular.z = theta_dot;

    return odom;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle nh("~");

    nh.getParam("frame", frame);
    nh.getParam("sensor_topic", sensor_topic);
    nh.getParam("odom_topic", odom_topic);
    nh.getParam("publish_rate", pub_rate);
    nh.getParam("pose_variance", pose_var);
    nh.getParam("velo_variance", velo_var);

    ros::Subscriber sub = nh.subscribe(sensor_topic, 5, data_callback);

    ros::Publisher odom_publish = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

    ros::Rate rate = ros::Rate(pub_rate);
    ros::Time last = ros::Time::now();

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time now = ros::Time::now();
        if (now - last > rate.expectedCycleTime())
        {
            last = now;
            auto msg = build_odom_packet();
            odom_publish.publish(msg);
        }
    }
}
