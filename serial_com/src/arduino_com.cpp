#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>  // Use the serial library (you can install it via `sudo apt install ros-<distro>-serial`)
#include <string>
#include <thread>

class SerialReaderNode : public rclcpp::Node {
public:
    SerialReaderNode() : Node("arduino_com"), latest_message_("") {
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

        // Start thread for reading serial data
        serial_thread_ = std::thread(&SerialReaderNode::readSerialData, this);
        serial_thread_.detach();
    }

    ~SerialReaderNode() {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    }

private:
    serial::Serial serial_port_;
    std::string latest_message_;
    std::thread serial_thread_;

    void readSerialData() {
        while (rclcpp::ok() && serial_port_.isOpen()) {
            try {
                std::string message = serial_port_.readline();
                if (!message.empty()) {
                    latest_message_ = message;
                    RCLCPP_INFO(this->get_logger(), "New message received: '%s'", latest_message_.c_str());
                }
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            }
        }
    }

    std::string getLatestMessage() const {
        return latest_message_;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
