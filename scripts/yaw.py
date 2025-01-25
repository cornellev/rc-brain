import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuYawPrinter(Node):
    def __init__(self):
        super().__init__('imu_yaw_printer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10  # QoS reliability depth
        )
        self.get_logger().info('IMU Yaw Printer Node Initialized')

    def imu_callback(self, msg):
        # Extract orientation quaternion
        orientation = msg.orientation
        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(orientation)
        self.get_logger().info(f'Yaw: {math.degrees(yaw):.4f} degrees')

    @staticmethod
    def quaternion_to_yaw(orientation):
        # Extract values from quaternion
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # Compute yaw from quaternion
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
