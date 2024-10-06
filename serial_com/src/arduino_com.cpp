#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <string>
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "cev_msgs/msg/sensor_collect.hpp" // Include your custom message header

class SerialHandlerNode : public rclcpp::Node {
public:
    SerialHandlerNode() : Node("serial_handler_node"), steering_angle_(0.0), velocity_(0.0), max_velocity_(2.07) {
        // Initialize serial port
        try {
            serial_port_.setPort("/dev/ttyACM0");
            serial_port_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();

            if (serial_port_.isOpen()) {
                RCLCPP_DEBUG(this->get_logger(), "Serial port initialized successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
        }

        // Subscribers
        rc_movement_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "rc_movement_msg", 1, std::bind(&SerialHandlerNode::rcMovementCallback, this, std::placeholders::_1));

        auto_max_vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "auto_max_vel", 1, std::bind(&SerialHandlerNode::maxVelocityCallback, this, std::placeholders::_1));

        // Publisher for sensor data
        sensor_collect_pub_ = this->create_publisher<cev_msgs::msg::SensorCollect>("sensor_collect", 1);

        // Timer for reading serial data periodically
        comm_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&SerialHandlerNode::commSerialData, this));
    }

private:
    serial::Serial serial_port_;
    rclcpp::TimerBase::SharedPtr comm_timer_;

    // Variables to store subscribed data
    float steering_angle_;
    float velocity_;
    float max_velocity_;

    // Variables for parsed serial data
    int32_t int_value_;
    float float_value1_, float_value2_;

    // Subscriber objects
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr auto_max_vel_sub_;
    
    // Publisher object for sensor data
    rclcpp::Publisher<cev_msgs::msg::SensorCollect>::SharedPtr sensor_collect_pub_;

    // Callback for the rc_movement_msg topic
    void rcMovementCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        steering_angle_ = msg->steering_angle;
        velocity_ = msg->speed;
    }

    // Callback for the auto_max_vel topic
    void maxVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        max_velocity_ = msg->data;
    }

    // Read and write serial data periodically
    void commSerialData() {
        if (serial_port_.isOpen()) {
            try {
                // Read a line (until newline character)
                std::string message = serial_port_.readline();

                if (!message.empty()) {
                    RCLCPP_DEBUG(this->get_logger(), "Raw message received: '%s'", message.c_str());

                    // Parse the received message into int and floats
                    if (parseMessage(message)) {
                        RCLCPP_DEBUG(this->get_logger(), "Parsed values - Int: %d, Float1: %f, Float2: %f",
                                    int_value_, float_value1_, float_value2_);

                        // Publish the parsed data
                        publishSensorData();
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "Failed to parse message: '%s'", message.c_str());
                    }
                }
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            }

            try {
                // Use ostringstream to control the precision of the float values
                std::ostringstream serial_message;
                serial_message << std::fixed << std::setprecision(2)
                            << velocity_ << " "
                            << steering_angle_ << " "
                            << max_velocity_ << "\n";

                // Send the message over the serial port
                serial_port_.write(serial_message.str());
                RCLCPP_DEBUG(this->get_logger(), "Serial message sent: '%s'", serial_message.str().c_str());

            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
            }
        }
    }

    // Helper function to parse incoming serial message
    bool parseMessage(const std::string& message) {
        size_t first_space = message.find(' ');
        size_t second_space = message.find(' ', first_space + 1);

        if (first_space == std::string::npos || second_space == std::string::npos) {
            return false; // Parsing failed
        }

        try {
            int_value_ = std::stoi(message.substr(0, first_space));
            float_value1_ = std::stof(message.substr(first_space + 1, second_space - first_space - 1));
            float_value2_ = std::stof(message.substr(second_space + 1));
            return true;
        } catch (const std::exception& e) {
            return false; // Parsing failed
        }
    }

    // Function to publish sensor data
    void publishSensorData() {
        auto sensor_msg = cev_msgs::msg::SensorCollect(); // Create message instance
        sensor_msg.timestamp = static_cast<uint32_t>(time(0)); // Get current timestamp
        sensor_msg.velocity = float_value1_; // Assuming float_value1_ holds velocity
        sensor_msg.steering_angle = float_value2_; // Assuming float_value2_ holds steering angle

        sensor_collect_pub_->publish(sensor_msg); // Publish the message
        RCLCPP_DEBUG(this->get_logger(), "Published sensor data: Timestamp: %u, Velocity: %f, Steering Angle: %f",
                     sensor_msg.timestamp, sensor_msg.velocity, sensor_msg.steering_angle);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
