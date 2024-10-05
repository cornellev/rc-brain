import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')

        self.initialized = False

        # Initialize the serial connection to the Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)  # Wait for the serial connection to initialize
            self.get_logger().info("Serial port initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial port: {e}")
            self.serial_port = None

        # Subscription to a string topic (can be extended for other types)
        self.subscription = self.create_subscription(
            String,
            '/input_topic',
            self.topic_callback,
            10
        )
    
    def topic_callback(self, msg):
        self.get_logger().info(f"Received message: '{msg.data}'")

        if self.serial_port and self.serial_port.isOpen():
            try:
                # Send the received message to the serial port
                message = msg.data + '\n'  # Add newline for Arduino compatibility
                self.serial_port.write(message.encode())
                self.get_logger().info("Message sent to serial port.")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")
        else:
            self.get_logger().error("Serial port is not open.")

def main(args=None):
    rclpy.init(args=args)

    node = SerialCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if node.serial_port:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
