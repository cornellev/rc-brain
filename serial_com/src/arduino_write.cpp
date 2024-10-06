#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <string>
#include <thread>
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

class SerialWriterNode : public rclcpp::Node {
public:
    SerialWriterNode() : Node("serial_writer_node"), steering_angle_(0.0), velocity_(0.0), max_velocity_(0.0) {
        // Initialize serial port
        try {
            serial_port_.setPort("/dev/ttyACM0");
            serial_port_.setBaudrate(9600);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
            
            if (serial_port_.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
        }

        // Subscribers
        rc_movement_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "rc_movement_msg", 10, std::bind(&SerialWriterNode::rcMovementCallback, this, std::placeholders::_1));

        auto_max_vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "auto_max_vel", 10, std::bind(&SerialWriterNode::maxVelocityCallback, this, std::placeholders::_1));

        // Start the thread to send serial messages periodically
        serial_thread_ = std::thread(&SerialWriterNode::sendSerialData, this);
        serial_thread_.detach();
    }

    ~SerialWriterNode() {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    }

private:
    serial::Serial serial_port_;
    std::thread serial_thread_;

    // Variables to store subscribed data
    float steering_angle_;
    float velocity_;
    float max_velocity_;

    // Subscriber objects
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr auto_max_vel_sub_;

    // Callback for the rc_movement_msg topic
    void rcMovementCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        steering_angle_ = msg->steering_angle;
        velocity_ = msg->speed;
    }

    // Callback for the auto_max_vel topic
    void maxVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        max_velocity_ = msg->data;
    }

    // Threaded function to send serial data periodically
    void sendSerialData() {
        while (rclcpp::ok() && serial_port_.isOpen()) {
            try {
                // Prepare the serial message in the format: float32 velocity, float32 steering_angle, float32 max_velocity
                std::string serial_message = std::to_string(velocity_) + " " +
                                            std::to_string(steering_angle_) + " " +
                                            std::to_string(max_velocity_) + "\n";
                
                // Send the message over the serial port
                serial_port_.write(serial_message);
                RCLCPP_INFO(this->get_logger(), "Serial message sent: '%s'", serial_message.c_str());

                // Sleep for 100ms before sending the next message
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialWriterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
