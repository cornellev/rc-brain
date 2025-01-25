#include <rclcpp/rclcpp.hpp>
#include <cev_msgs/msg/trajectory.hpp>
#include <cev_msgs/msg/waypoint.hpp>
#include <vector>
#include <chrono>

using cev_msgs::msg::Trajectory;
using cev_msgs::msg::Waypoint;

// ROS2 Node that publishes a cev_msgs trajectory
class DebugTrajectoryPublish : public rclcpp::Node {
public:
    DebugTrajectoryPublish(): Node("debug_trajectory_publish") {
        trajectory_pub_ = this->create_publisher<cev_msgs::msg::Trajectory>("trajectory", 1);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000),
            std::bind(&DebugTrajectoryPublish::timer_callback, this));
    }

private:
    rclcpp::Publisher<cev_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<std::tuple<float, float, float>>> trajectories = {
        {
            {0.0, 0.0, 0.9},
            {0.3, 0.1, 0.7},
            {0.6, 0.2, 0.5},
            {0.9, 0.3, 0.3},
            {1.2, 0.4, 0.2},
            {1.5, 0.5, 0.2},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3}
        },
        {
            {0.6, 0.2, 0.5},
            {0.9, 0.3, 0.3},
            {1.2, 0.4, 0.2},
            {1.5, 0.5, 0.2},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3},
        },
        {
            {1.2, 0.4, 0.2},
            {1.5, 0.5, 0.2},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3},
            {0.6, 0.5, -0.3},
            {1.5, -0.5, 0.3},
            {0.0, 0.0, -.3},
        }
    };

    size_t current_trajectory = 0;

    void add_waypoints(Trajectory &trajectory, std::vector<std::tuple<float, float, float>> waypoints) {
        for (auto &waypoint : waypoints) {
            add_waypoint(trajectory, std::get<0>(waypoint), std::get<1>(waypoint), std::get<2>(waypoint));
        }
    }

    void add_waypoint(Trajectory &trajectory, float x, float y, float v) {
        Waypoint waypoint = Waypoint();
        waypoint.x = x;
        waypoint.y = y;
        waypoint.v = v;
        trajectory.waypoints.push_back(waypoint);
    }

    void timer_callback() {
        if (current_trajectory < trajectories.size()) {
            Trajectory trajectory_msg = Trajectory();
            trajectory_msg.header.stamp = this->now();
            add_waypoints(trajectory_msg, trajectories[current_trajectory]);
            trajectory_pub_->publish(trajectory_msg);
            current_trajectory++;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugTrajectoryPublish>());
    rclcpp::shutdown();
    return 0;
}