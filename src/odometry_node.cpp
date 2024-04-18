#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rc_localization/SensorCollect.h>

const double BIKE_LENGTH = 19.0 / 100;
const double WHEEL_DIAMETER_M = 9.5 / 100;
const double TICKS_PER_REV = 827.2;
const double TICKS_TO_M = (1 / TICKS_PER_REV) * (2 * M_PI * (WHEEL_DIAMETER_M / 2));

int initial_encoder_left;
int initial_encoder_right;

bool first = true;
rc_localization::SensorCollect last;

double steer_angle = 0.0;
double x, y, theta, x_dot, y_dot, theta_dot = 0;

void data_callback(rc_localization::SensorCollect current)
{
    // Calculate encoder offset
    current.encoder_left -= initial_encoder_left;
    current.encoder_right -= initial_encoder_right;

    if (first)
    {
        first = false;

        // Zero the encoders on startup
        initial_encoder_left = current.encoder_left;
        initial_encoder_right = current.encoder_right;
        last = current;
        return;
    }

    ros::Duration dt = ros::Duration((current.timestamp - last.timestamp) / 1000.0);

    double current_avg = (current.encoder_left + current.encoder_right) / 2.0;
    double last_avg = (last.encoder_left + last.encoder_right) / 2.0;
    double delta_enc = current_avg - last_avg;
    double delta_enc_m = TICKS_TO_M * delta_enc;

    last = current;

    double v = delta_enc_m / dt.toSec();
    double steer_angle = current.steering_angle;

    // should be new theta?
    x_dot = v * std::cos(theta);
    y_dot = v * std::sin(theta);
    theta_dot = v * std::tan(steer_angle) / BIKE_LENGTH;

    // Don't update theta for now, at least until we switch to getting an initial estimate of theta.
    x += x_dot * dt.toSec();
    y += y_dot * dt.toSec();
    theta += theta_dot * dt.toSec(); // temporary
}

void state_callback(nav_msgs::Odometry filtered)
{
    tf2::Quaternion q;
    tf2::fromMsg(filtered.pose.pose.orientation, q);

    tf2::Matrix3x3 m(q);

    double yaw, _pitch, _roll;
    m.getEulerYPR(yaw, _pitch, _roll);

    theta = yaw;
}

nav_msgs::Odometry get_odom_packet()
{
    nav_msgs::Odometry odom;

    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setEuler(theta, 0, 0);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = x_dot;
    odom.twist.twist.linear.y = y_dot;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.z = theta_dot;

    return odom;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    ros::NodeHandle nh;

    ros::Subscriber s1 = nh.subscribe("/sensor_collect", 5, data_callback);
    ros::Subscriber s2 = nh.subscribe("/odometry/filtered", 5, state_callback);

    ros::Publisher odom_publish = nh.advertise<nav_msgs::Odometry>("/odometry", 1);

    ros::Rate rate = ros::Rate(100);
    ros::Time last = ros::Time::now();

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        if (now - last > rate.expectedCycleTime())
        {
            last = now;
            auto msg = get_odom_packet();
            odom_publish.publish(msg);
        }

        ros::spinOnce();
    }
}
