#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/waypoint.hpp"
#include "cev_msgs/msg/trajectory.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using cev_msgs::msg::Waypoint;
using std::placeholders::_1;

struct Coordinate {
    Coordinate(float x, float y, float theta, float v): x(x), y(y), theta(theta), v(v) {}

    float x;
    float y;
    float theta;
    float v;
};

class TrajectoryFollower : public rclcpp::Node {
public:
    TrajectoryFollower()
        : Node("trajectory_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 1,
            std::bind(&TrajectoryFollower::odometry_callback, this, _1));

        trajectory_sub_ = this->create_subscription<cev_msgs::msg::Trajectory>("trajectory", 1,
            std::bind(&TrajectoryFollower::trajectory_callback, this, _1));

        ackermann_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg", 1);
    }

private:
    float min_steering_angle = -20.0 * M_PI / 180.0;
    float max_steering_angle = 20.0 * M_PI / 180.0;

    float waypoint_radius = .1;
    float waypoint_final_radius = .1;

    bool waypoints_initialized = false;

    std::vector<Waypoint> waypoints;
    size_t current_waypoint = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<cev_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    float normalize_angle(float f) {
        float modded = fmod(f, 2 * M_PI);

        if (modded >= M_PI) {
            modded -= 2 * M_PI;
        } else if (modded < -M_PI) {
            modded += 2 * M_PI;
        }

        return modded;
    }

    float dist_to_waypoint(Coordinate& current, Waypoint& target) {
        return std::sqrt(std::pow(current.x - target.x, 2) + std::pow(current.y - target.y, 2));
    }

    float angle_to_waypoint(Coordinate& current, Waypoint& target) {
        float theta = current.theta;

        // RCLCPP_INFO(this->get_logger(), "Current Angle: %f", current.theta);

        float multiplier = 1.0;

        if (target.v < 0) {
            theta = normalize_angle(theta + M_PI);
            multiplier = -1.0;
        }

        float output_angle =
            multiplier
            * normalize_angle(std::atan2(target.y - current.y, target.x - current.x) - theta);

        // RCLCPP_INFO(this->get_logger(), "ANGLE DIFF: %f", output_angle);

        // if (output_angle > M_PI) {
        //     output_angle -= 2 * M_PI;
        // } else if (output_angle < -M_PI) {
        //     // output_angle = -(2 * M_PI - output_angle);
        //     output_angle += 2 * M_PI;
        // }

        return output_angle;
    }

    bool waypoint_reached(Coordinate& current, size_t waypoint_idx) {
        Waypoint target = waypoints[waypoint_idx];

        float dist = dist_to_waypoint(current, target);
        bool final = waypoint_idx == waypoints.size() - 1;
        float radius = final ? waypoint_final_radius : waypoint_radius;

        if (dist < radius) {
            return true;
        }

        // Check if passed line seg
        float dot;

        if (final) {
            Waypoint prev = waypoints[waypoint_idx - 1];
            dot = (current.x - target.x) * (target.x - prev.x)
                  + (current.y - target.y) * (target.y - prev.y);
        } else {
            Waypoint next = waypoints[waypoint_idx + 1];
            dot = (current.x - target.x) * (next.x - target.x)
                  + (current.y - target.y) * (next.y - target.y);
        }

        return dist < .1 && dot > 0;
    }

    float find_steering_angle(Coordinate& current, Waypoint& target) {
        return angle_to_waypoint(current, target);
    }

    void publish_ackermann_drive(float steering_angle, float speed) {
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
        ackermann_msg.steering_angle = normalize_angle(steering_angle);

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

        geometry_msgs::msg::TransformStamped transform;

        geometry_msgs::msg::PoseStamped base_link_pose;
        base_link_pose.header = msg->header;
        base_link_pose.pose = msg->pose.pose;

        geometry_msgs::msg::PoseStamped map_pose;

        try {
            transform = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
            tf2::doTransform(base_link_pose, map_pose, transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
            return;
        }

        float qw = map_pose.pose.orientation.w;
        float qx = map_pose.pose.orientation.x;
        float qy = map_pose.pose.orientation.y;
        float qz = map_pose.pose.orientation.z;

        float yaw = normalize_angle(atan2(2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)));

        Coordinate current = Coordinate(map_pose.pose.position.x, map_pose.pose.position.y, yaw,
            msg->twist.twist.linear.x);

        // Skip reached waypoints
        while (waypoint_reached(current, current_waypoint)) {
            current_waypoint++;
            if (current_waypoint >= waypoints.size()) {
                waypoints_initialized = false;
                publish_ackermann_drive(0.0, 0.0);
                return;
            }
        }

        Waypoint target = waypoints[current_waypoint];

        // RCLCPP_INFO(this->get_logger(), "Target: %f, %f", target.x, target.y);

        float steering_angle = find_steering_angle(current, target);

        // RCLCPP_INFO(this->get_logger(), "Target Steering: %f", steering_angle);

        float vel = 0;

        if (target.v > .01 && target.v < .2) {
            target.v = .2;
        } else if (target.v < -.01 && target.v > -.2) {
            target.v = -.2;
        }

        // publish_ackermann_drive(steering_angle, vel);
        publish_ackermann_drive(target.tau, target.v);
    }

    void trajectory_callback(const cev_msgs::msg::Trajectory::SharedPtr msg) {
        waypoints = msg->waypoints;
        current_waypoint = 0;
        waypoints_initialized = true;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}
