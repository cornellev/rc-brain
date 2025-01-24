#include <rclcpp/rclcpp.hpp>
#include <cev_msgs/msg/trajectory.hpp>
#include <cev_msgs/msg/waypoint.hpp>
#include <vector>
#include <chrono>


// ROS2 Node that publishes a cev_msgs trajectory
class DebugTrajectoryPublish : public rclcpp::Node {
public:
    DebugTrajectoryPublish(): Node("debug_trajectory_publish") {
        trajectory_pub_ = this->create_publisher<cev_msgs::msg::Trajectory>("trajectory", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
            std::bind(&DebugTrajectoryPublish::timer_callback, this));
    }

private:
    rclcpp::Publisher<cev_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void add_waypoint(cev_msgs::msg::Trajectory &trajectory, float x, float y, float v) {
        auto waypoint = cev_msgs::msg::Waypoint();
        waypoint.x = x;
        waypoint.y = y;
        waypoint.v = v;
        trajectory.waypoints.push_back(waypoint);
    }

    void timer_callback() {
        auto trajectory_msg = cev_msgs::msg::Trajectory();
        trajectory_msg.header.stamp = this->now();

        add_waypoint(trajectory_msg, 0.0, 0.0, 0.9);
        add_waypoint(trajectory_msg, 0.3, 0.1, 0.7);
        add_waypoint(trajectory_msg, 0.6, 0.2, 0.5);
        add_waypoint(trajectory_msg, 0.9, 0.3, 0.3);
        add_waypoint(trajectory_msg, 1.2, 0.4, 0.2);
        add_waypoint(trajectory_msg, 1.5, 0.5, 0.1);

        trajectory_pub_->publish(trajectory_msg);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugTrajectoryPublish>());
    rclcpp::shutdown();
    return 0;
}