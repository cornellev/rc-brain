#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>
#include <vector>

class LogitechRead {
public:
    LogitechRead(const std::vector<float>& axes, const std::vector<int>& buttons)
        : axes_(axes), buttons_(buttons) {}

    float get_left_trigger() const {
        return 1 - (axes_[2] + 1) / 2;
    }

    float get_right_trigger() const {
        return 1 - (axes_[5] + 1) / 2;
    }

    float get_y_button() const {
        return buttons_[3];
    }

    float get_right_stick_x() const {
        return -axes_[3];
    }

private:
    std::vector<float> axes_;
    std::vector<int> buttons_;
};

class JoyInterpreter : public rclcpp::Node {
public:
    JoyInterpreter(): Node("joy_interpreter") {
        movement_pub_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg", 1);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
            std::bind(&JoyInterpreter::joy_to_twist, this, std::placeholders::_1));

        this->declare_parameter<double>("joy_interpreter/teleop/deadzone", 0.1);
        this->declare_parameter<double>("joy_interpreter/teleop/max_turning_angle",
            M_PI / 9);  // 20 degrees in radians
        this->declare_parameter<double>("joy_interpreter/teleop/max_velocity", 1.5);

        deadzone_ = this->get_parameter("joy_interpreter/teleop/deadzone").as_double();
        max_turning_angle_ =
            this->get_parameter("joy_interpreter/teleop/max_turning_angle").as_double();
        max_velocity_ = this->get_parameter("joy_interpreter/teleop/max_velocity").as_double();

        RCLCPP_INFO(this->get_logger(), "Parameter max_turning_angle=%f", max_turning_angle_);
        RCLCPP_INFO(this->get_logger(), "Parameter max_velocity=%f", max_velocity_);
        RCLCPP_INFO(this->get_logger(), "Parameter deadzone=%f", deadzone_);
    }

private:
    void joy_to_twist(const sensor_msgs::msg::Joy::SharedPtr msg) {
        LogitechRead gamepad_data(msg->axes, msg->buttons);

        // Calculate turning angle
        float turn = -gamepad_data.get_right_stick_x() * max_turning_angle_;
        if (std::abs(turn) < deadzone_) {
            turn = 0.0;
        }

        // Calculate velocity
        float drive = 0.0;

//        if (gamepad_data.get_y_button()) {
//            drive = .3;
//        } else {
            drive = (gamepad_data.get_right_trigger() - gamepad_data.get_left_trigger())
                    * max_velocity_;
//        }

        // float drive = (gamepad_data.get_right_trigger() - gamepad_data.get_left_trigger())
        //               * max_velocity_;

        // float drive = gamepad_data.get_y_button() ? .3 : 0.0;

        // RCLCPP_INFO(this->get_logger(), "Button: %f", gamepad_data.get_y_button());

        // Publish AckermannDrive message
        auto ackermann_msg = ackermann_msgs::msg::AckermannDrive();
        ackermann_msg.steering_angle = turn;
        ackermann_msg.speed = drive;
        movement_pub_->publish(ackermann_msg);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr movement_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    double deadzone_;
    double max_turning_angle_;
    double max_velocity_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyInterpreter>());
    rclcpp::shutdown();
    return 0;
}
