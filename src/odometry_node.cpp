#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double BIKE_LENGTH = 10.0; // TODO
const double WHEEL_DIAMETER_M = 110.0 / 1000;
const double TICKS_PER_REV = 827.2;
const double TICKS_TO_M = TICKS_PER_REV * 2 * M_PI * (WHEEL_DIAMETER_M / 2);

struct EncoderData
{
    ros::Time stamp;
    int32_t data;

    EncoderData()
    {
    }

    EncoderData(ros::Time stamp, int32_t data) : stamp(stamp), data(data)
    {
    }
};

EncoderData left_enc_last, left_enc_current, right_enc_last, right_enc_current;
double steer_angle = 0.0;
double x, y, theta, x_dot, y_dot, theta_dot;

// these really need to be stamped
void left_enc_callback(std_msgs::Int32 encoder)
{
    left_enc_last = left_enc_current;
    left_enc_current = EncoderData(ros::Time::now(), encoder.data);
}

void right_enc_callback(std_msgs::Int32 encoder)
{
    right_enc_last = right_enc_current;
    right_enc_current = EncoderData(ros::Time::now(), encoder.data);
}

void steer_angle_callback(std_msgs::Float32 angle)
{
    steer_angle = angle.data;
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

void calculate_bicycle(ros::Time last_time, ros::Time now)
{
    ros::Duration dt = now - last_time;

    double v_left = (left_enc_current.data - left_enc_last.data) /
                    (left_enc_current.stamp - left_enc_last.stamp).toSec();
    double v_right = (right_enc_current.data - right_enc_last.data) /
                     (right_enc_current.stamp - right_enc_last.stamp).toSec();

    // Interpolate/extrapolate encoder position to last calculation time and current calculation time
    double left_old = left_enc_last.data + v_left * (last_time - left_enc_last.stamp).toSec();
    double left_now = left_enc_current.data + v_left * (now - left_enc_current.stamp).toSec();
    double left_delta = left_now - left_old;

    double right_old = right_enc_last.data + v_right * (last_time - right_enc_last.stamp).toSec();
    double right_now = right_enc_current.data + v_right * (now - right_enc_current.stamp).toSec();
    double right_delta = right_now - right_old;

    double v = (left_delta + right_delta) / 2 * dt.toSec();

    // should be new theta?
    x_dot = v * std::cos(theta);
    y_dot = v * std::sin(theta);
    theta_dot = v * std::tan(steer_angle) / BIKE_LENGTH;

    x += x_dot * dt.toSec();
    y += y_dot * dt.toSec();
    // Don't update theta for now, at least until we switch to getting an initial estimate of theta.
}

nav_msgs::Odometry get_odom_packet(ros::Time stamp)
{
    nav_msgs::Odometry odom;

    odom.header.frame_id = "odom";
    odom.header.stamp = stamp;

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

    ros::NodeHandle nh("~");

    nh.subscribe("/encoder/left", 5, left_enc_callback);
    nh.subscribe("/encoder/right", 5, right_enc_callback);
    nh.subscribe("/steering_angle", 5, steer_angle_callback);
    nh.subscribe("/odometry/filtered", 5, state_callback);

    ros::Publisher odom_publish = nh.advertise<nav_msgs::Odometry>("/odometry", 5);

    ros::Time last_calc = ros::Time::now();
    ros::Rate rate = ros::Rate(10);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        calculate_bicycle(last_calc, now);
        last_calc = now;

        auto msg = get_odom_packet(now);
        odom_publish.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
}