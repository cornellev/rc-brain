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
    TrajectoryFollower() : Node("trajectory_follower") {
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 1, std::bind(&TrajectoryFollower::odometry_callback, this, _1)
        );

        trajectory_sub_ = this->create_subscription<cev_msgs::msg::Trajectory>(
            "trajectory", 1, std::bind(&TrajectoryFollower::trajectory_callback, this, _1)
        );
    }
private:
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    std::vector<cev_msgs::msg::Waypoint> waypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<cev_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg = msg;
    }

    void trajectory_callback(const cev_msgs::msg::Trajectory::SharedPtr msg) {
        waypoints = msg->waypoints;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}