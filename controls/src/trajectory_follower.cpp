#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/waypoint.hpp"
#include "cev_msgs/msg/trajectory.hpp"
#include <vector>
#include <cmath>

using std::placeholders::_1;
using cev_msgs::msg::Waypoint;

struct Coordinate {
    Coordinate(float x, float y, float theta, float v): x(x), y(y), theta(theta), v(v) {}

    float x;
    float y;
    float theta;
    float v;
};

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
    float waypoint_final_radius = .3;

    bool waypoints_initialized = false;
    std::vector<Waypoint> waypoints;
    size_t current_waypoint = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<cev_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;

    float angle_mod(float f) {
        return fmod(f, 2 * M_PI);
    }

    float dist_to_waypoint(Coordinate& current, Waypoint& target) {
        return std::sqrt(std::pow(current.x - target.x, 2) + std::pow(current.y - target.y, 2));
    }

    float angle_to_waypoint(Coordinate& current, Waypoint& target) {
        RCLCPP_INFO(this->get_logger(), "ANGLELLEELEL: %f", angle_mod(std::atan2((target.y - current.y), (target.x - current.x))));

        return angle_mod(angle_mod(std::atan2((target.y - current.y), (target.x - current.x))) - current.theta);
    }

    bool check_waypoint_reached(Coordinate& current, Waypoint& target) {
        float dist = dist_to_waypoint(current, target);
        bool final = current_waypoint == waypoints.size() - 1;
        float radius = final ? waypoint_final_radius : waypoint_radius;

        if (dist < radius) {
            RCLCPP_INFO(this->get_logger(), "CASE 1 : %zu : %f < %f", current_waypoint, dist, radius);
            return true;
        }

        // return false;

        // Check if passed line seg
        float dot;

        if (final) {
            Waypoint next = waypoints[current_waypoint - 1];
            dot = (current.x - target.x) * (target.x - next.x) + (current.y - target.y) * (target.y -  next.y);
            if (dot > 0) {
                RCLCPP_INFO(this->get_logger(), "CASE 2 : %zu", current_waypoint);
            }
        } else {
            Waypoint next = waypoints[current_waypoint + 1];
            dot = (current.x - target.x) * (next.x - target.x) + (current.y - target.y) * (next.y - target.y);
            if (dot > 0) {
                RCLCPP_INFO(this->get_logger(), "CASE 3 : %zu", current_waypoint);
            }
        }

        return dist < .3 && dot > 0;

        // return dot > 0;
        // return false;
    }

    float find_steering_angle(Coordinate& current, Waypoint& target) {
        float s = dist_to_waypoint(current, target);
        float alpha = angle_to_waypoint(current, target);

        // return std::atan(2 * WB * std::sin(alpha) / s);
        return alpha;
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
        if (!waypoints_initialized || current_waypoint >= waypoints.size()) {
            return;
        }

        Coordinate current = Coordinate(
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y,
            angle_mod(2 * std::acos(msg->pose.pose.orientation.w)), 
            msg->twist.twist.linear.x
        );

        Waypoint target = waypoints[current_waypoint];

        // Skip reached waypoints
        if (check_waypoint_reached(current, target)) {
            current_waypoint++;
            if (current_waypoint >= waypoints.size()) {
                waypoints_initialized = false;
                publish_ackermann_drive(0.0, 0.0);
                return;
            }
            target = waypoints[current_waypoint];
        }

        float steering_angle = find_steering_angle(current, target);

        RCLCPP_INFO(
            this->get_logger(),
            "Target is: (%f, %f). Steering angle is: %f",
            target.x, target.y, steering_angle
        );

        // RCLCPP_INFO(this->get_logger(), "Target Steering %zu", current_waypoint);

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