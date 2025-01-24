#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/waypoint.hpp"
#include "cev_msgs/msg/trajectory.hpp"

using std::placeholders::_1;

struct Waypoint {
    Waypoint(double x, double y, double v): x(x), y(y), v(v) {};

    double x;
    double y;
    double v;
};

struct LineSegment {
    LineSegment(double x0, double y0, double x1, double y1): x0(x0), y0(y0), x1(x1), y1(y1) {};

    double x0;
    double y0;
    double x1;
    double y1;
};

class TrajectoryFollower : public rclcpp::Node {
public:
    TrajectoryFollower(): Node("trajectory_follower") {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 1,
            std::bind(&TrajectoryFollower::odometry_callback, this, _1));

        trajectory_sub_ = this->create_subscription<cev_msgs::msg::Trajectory>("trajectory", 1,
            std::bind(&TrajectoryFollower::trajectory_callback, this, _1));

        ackermann_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg", 1);
    }

private:
    double min_steering_angle = -20.0 * M_PI / 180.0;
    double max_steering_angle = 20.0 * M_PI / 180.0;

    double waypoint_radius = 0.3;

    bool waypoints_initialized = false;
    std::vector<Waypoint> waypoints;
    size_t current_target_wp = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<cev_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_pub_;

    double dist_to_waypoint(double current_x, double current_y, double target_x, double target_y) {
        return std::sqrt(std::pow(current_x - target_x, 2) + std::pow(current_y - target_y, 2));
    }

    double angle_to_waypoint(double current_x, double current_y, double current_theta,
        double target_x, double target_y) {
        return std::atan2(target_y - current_y, target_x - current_x) - current_theta;
    }

    LineSegment track_segment_beginning_at(size_t waypoint) {
        Waypoint wp_next = waypoints[waypoint + 1];
        Waypoint wp = waypoints[waypoint];

        return LineSegment(wp.x, wp.y, wp_next.x, wp_next.y);
    }

    bool check_waypoint_reached(double current_x, double current_y) {
        Waypoint wp = waypoints[current_target_wp];
        double dist = std::sqrt(std::pow(current_x - wp.x, 2) + std::pow(current_y - wp.y, 2));
        if (dist < waypoint_radius) {
            return true;
        }

        // TODO: current_target_wp = end
        LineSegment segment = track_segment_beginning_at(current_target_wp);

        double rel_x = current_x - segment.x0;
        double rel_y = current_x - segment.y0;
        double norm_x = segment.x1 - segment.x0;
        double norm_y = segment.y1 - segment.y0;

        double dot = rel_x * norm_x + rel_y * norm_y;

        if (dot > 0) {
            return true;
        }

        return false;
    }

    void publish_ackermann_drive(double steering_angle, double speed) {
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
            publish_ackermann_drive(0.0, 0.0);
            return;
        }

        if (check_waypoint_reached(msg->pose.pose.position.x, msg->pose.pose.position.y)) {
            current_target_wp++;
            if (current_target_wp >= waypoints.size()) {
                waypoints_initialized = false;
                return;
            }
        }

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double theta = 2 * std::acos(msg->pose.pose.orientation.w);

        Waypoint target = waypoints[current_target_wp];
    }

    void trajectory_callback(const cev_msgs::msg::Trajectory::SharedPtr msg) {
        if (msg->waypoints.size() < 2) {
            RCLCPP_WARN(this->get_logger(),
                "Invalid trajectory. Must contain at least 2 points but had %zu.",
                msg->waypoints.size());
            return;
        }

        waypoints.clear();
        waypoints.reserve(msg->waypoints.size());
        for (const auto& wp_msg: msg->waypoints) {
            waypoints.emplace_back(wp_msg.x, wp_msg.y, wp_msg.v);
        }

        current_target_wp = 0;
        waypoints_initialized = true;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}