#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/waypoint.hpp"
#include "cev_msgs/msg/trajectory.hpp"
#include <vector>
#include <cmath>

using std::placeholders::_1;

class TrajectoryFollower : public rclcpp::Node {
public:
    TrajectoryFollower(): Node("trajectory_follower") {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 1,
            std::bind(&TrajectoryFollower::odometry_callback, this, _1));

        trajectory_sub_ = this->create_subscription<cev_msgs::msg::Trajectory>("trajectory", 1,
            std::bind(&TrajectoryFollower::trajectory_callback, this, _1));

        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg", 1);
    }

private:
    float WB = 0.185;

    float min_steering_angle = -20.0 * M_PI / 180.0;
    float max_steering_angle = 20.0 * M_PI / 180.0;

    float waypoint_radius = 0.3;

    bool waypoints_initialized = false;
    std::vector<cev_msgs::msg::Waypoint> waypoints;
    int current_waypoint = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<cev_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;

    float dist_to_waypoint(float current_x, float current_y, float target_x, float target_y) {
        return std::sqrt(std::pow(current_x - target_x, 2) + std::pow(current_y - target_y, 2));
    }

    float angle_to_waypoint(float current_x, float current_y, float current_theta, float target_x, float target_y) {
        return std::atan2(target_y - current_y, target_x - current_x) - current_theta;
    }

    bool check_waypoint_reached(float current_x, float current_y, float target_x, float target_y) {
        return dist_to_waypoint(current_x, current_y, target_x, target_y) < waypoint_radius;
    }

    float find_steering_angle(float current_x, float current_y, float current_theta, float target_x, float target_y) {
        float s = dist_to_waypoint(current_x, current_y, target_x, target_y);
        float alpha = angle_to_waypoint(current_x, current_y, current_theta, target_x, target_y);

        return std::atan2(2 * WB * std::sin(alpha) / s, 1.0);
    }

    void publish_ackermann_drive(float steering_angle, float speed) {
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
        ackermann_msg.steering_angle = fmod(steering_angle, 2 * M_PI);

        if (ackermann_msg.steering_angle < min_steering_angle) {
            ackermann_msg.steering_angle = min_steering_angle;
        } else if (ackermann_msg.steering_angle > max_steering_angle) {
            ackermann_msg.steering_angle = max_steering_angle;
        }

        ackermann_msg.speed = speed;

        ackermann_pub_->publish(ackermann_msg);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!waypoints_initialized) {
            return;
        }

        if (check_waypoint_reached(msg->pose.pose.position.x, msg->pose.pose.position.y,
            waypoints[current_waypoint].x, waypoints[current_waypoint].y)) {
            current_waypoint++;
            if (current_waypoint >= waypoints.size()) {
                waypoints_initialized = false;
                publish_ackermann_drive(0.0, 0.0);
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", current_waypoint);

        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float theta = 2 * std::acos(msg->pose.pose.orientation.w);
        float v = msg->twist.twist.linear.x;

        cev_msgs::msg::Waypoint target = waypoints[current_waypoint];

        float steering_angle = find_steering_angle(x, y, theta, target.x, target.y);

        publish_ackermann_drive(steering_angle, target.v);
    }

    void trajectory_callback(const cev_msgs::msg::Trajectory::SharedPtr msg) {
        if (!waypoints_initialized) {
            waypoints_initialized = true;
        }

        waypoints = msg->waypoints;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}