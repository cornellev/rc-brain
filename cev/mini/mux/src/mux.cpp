#include "rclcpp/rclcpp.hpp"
#include <string>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <iostream>

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

class MuxNode : public rclcpp::Node {
public:
    MuxNode(): Node("mux_node") {
        RCLCPP_INFO(this->get_logger(), "Starting Mux initialization.");

        // Subscribers
        teleop_sub = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("teleop_msg", 1,
            std::bind(&MuxNode::teleopCallback, this, std::placeholders::_1));

        autonomous_sub =
            this->create_subscription<ackermann_msgs::msg::AckermannDrive>("autonomous_msg", 1,
                std::bind(&MuxNode::autonomousCallback, this, std::placeholders::_1));

        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1,
            std::bind(&MuxNode::joyCallback, this, std::placeholders::_1));

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg",
            1);

        RCLCPP_INFO(this->get_logger(), "Mux initialized successfully.");
    }

private:
    // Subscribed topics to send over serial
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr teleop_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr autonomous_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_pub;

    int debounce = 0;

    bool teleop_active = true;

    void teleopCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        if (teleop_active) {
            drive_pub->publish(*msg);
        }
    }

    void autonomousCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        if (!teleop_active) {
            drive_pub->publish(*msg);
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        LogitechRead gamepad_data(msg->axes, msg->buttons);

        if (gamepad_data.get_y_button()) {
            if (debounce == 0) {
                debounce = 1;
            } else if (debounce == 1) {
                teleop_active = !teleop_active;
                debounce = 2;
            } else {
                debounce = 2;
            }
        } else {
            if (debounce == 2) {
                debounce = 3;
            } else {
                debounce = 0;
            }
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MuxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
