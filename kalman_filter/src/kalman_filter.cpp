#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

using std::placeholders::_1;

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("encoder_odometry") {
        RCLCPP_INFO(this->get_logger(), "Initializing Kalman Filter Node");
    }

private:
    const float WHEELBASE = 0.3;

    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Zero();

    rclcpp::Time last_update_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}