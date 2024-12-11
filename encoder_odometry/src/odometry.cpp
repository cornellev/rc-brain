#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "cev_msgs/msg/sensor_collect.hpp"
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode(): Node("encoder_odometry") {
        RCLCPP_INFO(this->get_logger(), "Initializing Ackermann Odometry Node");

        sensor_collect_sub_ = this->create_subscription<cev_msgs::msg::SensorCollect>(
            "sensor_collect", 1, std::bind(&OdometryNode::updateOdometry, this, _1));

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/encoder_odometry", 1);
    }

private:
    const float WHEELBASE = 0.185;

    double x_ = 0.0;  // Forward of base_link is x
    double y_ = 0.0;

    double yaw_ = 0.0;

    double old_left_rotations_ = 0.0;
    double new_left_rotations_ = 0.0;

    double old_steering_angle_ = 0.0;

    rclcpp::Time prev_time_;

    bool initialized = false;

    double covariance_[36] = {0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

    rclcpp::Subscription<cev_msgs::msg::SensorCollect>::SharedPtr sensor_collect_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    void updateOdometry(const cev_msgs::msg::SensorCollect::SharedPtr data) {
        if (!initialized) {
            prev_time_ = this->now();
            initialized = true;
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - prev_time_).seconds();
        if (dt < 1e-6) return;

        double delta_distance = data->velocity * dt;

        double new_steering_angle = data->steering_angle;
        double average_steering_angle = new_steering_angle;
        // double average_steering_angle = (new_steering_angle + old_steering_angle_) / 2.0;

        double turning_radius = WHEELBASE / std::tan(average_steering_angle);

        double delta_theta = delta_distance / turning_radius;

        yaw_ += delta_theta;
        x_ += delta_distance * std::cos(yaw_);
        y_ += delta_distance * std::sin(yaw_);

        publishOdometry(this->now(), data->velocity, delta_theta / dt);

        // Update carry vars
        old_steering_angle_ = new_steering_angle;
        prev_time_ = current_time;
    }

    void publishOdometry(const rclcpp::Time& current_time, double linear_velocity,
        double angular_velocity) {
        /**
         * Publish odometry message using current pose/twist along with calculated velocities.
         */
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_velocity;
        odom_msg.twist.twist.angular.z = angular_velocity;

        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = covariance_[i];
        }

        odom_publisher_->publish(odom_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
