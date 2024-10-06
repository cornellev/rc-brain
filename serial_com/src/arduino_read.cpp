#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>  // Use the serial library
#include <string>
#include <sstream>
#include <thread>

class SerialReaderNode : public rclcpp::Node {
public:
    SerialReaderNode() : Node("serial_reader_node") {
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
    std::thread serial_thread_;

    // Variables to store parsed data
    int32_t int_value_;
    float float_value1_;
    float float_value2_;

    void readSerialData() {
        while (rclcpp::ok() && serial_port_.isOpen()) {
            try {
                // Read a line (until newline character)
                std::string message = serial_port_.readline();
                
                if (!message.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Raw message received: '%s'", message.c_str());

                    // Parse the received message into int and floats
                    if (parseMessage(message)) {
                        RCLCPP_INFO(this->get_logger(), "Parsed values - Int: %d, Float1: %f, Float2: %f",
                                    int_value_, float_value1_, float_value2_);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Failed to parse message: '%s'", message.c_str());
                    }
                }
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            }
        }
    }

    bool parseMessage(const std::string& message) {
        std::istringstream iss(message);
        // Attempt to parse the message into the expected format: int32 float32 float32
        if (iss >> int_value_ >> float_value1_ >> float_value2_) {
            return true;
        } else {
            return false;  // Parsing failed
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
