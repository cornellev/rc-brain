from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch(package, file, launch_folder="launch"):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                launch_folder,
                file
            )
        )
    )

def generate_launch_description():
    imu_config = os.path.join(
        get_package_share_directory('autonomy'),
        'config',
        'imu.yml'
    )

    return LaunchDescription(
        [
            # STATIC TRANSFORMS
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_world_map',
                output='screen',
                arguments=[
                    '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                    '0', '0', '0', '1', # Rotation: M_PI
                    'world',
                    'map'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_map_odom',
                output='screen',
                arguments=[
                    '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                    '0', '0', '0', '1', # Rotation: M_PI
                    'map',
                    'odom'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_odom_base_link',
                output='screen',
                arguments=[
                    '0', '0', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                    '0', '0', '0', '1', # Rotation: M_PI
                    'odom',
                    'base_link'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_base_link_lidar',
                output='screen',
                arguments=[
                    '-0.035', '0.04', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                    '0', '0', '1', '0', # Rotation: M_PI
                    'base_link',
                    'laser_frame'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_base_link_imu',
                output='screen',
                arguments=[
                    '-0.18', '0.07', '0',  # Translation: x = -0.18, y = 0.07, z = 0 (meters)
                    '0', '0', '0', '1', # Rotation: 0
                    'base_link',
                    'imu'
                ]
            ),
            # Run joystick reader
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[] #ros2 uses events, so don't try and direct this to /dev
            ),
            # IMU
            Node(
                package='witmotion_ros',
                executable='witmotion_ros_node',
                parameters=[imu_config]
            ),

            ## LAUNCH FILES

            # Teleop
            launch("teleop", "launch.py"),
            # Autobrake
            launch("controls", "autobrake_launch.py"),
            # Serial Communicator
            launch("serial_com", "launch.py"),
            # RPLidar
            launch("sllidar_ros2", "sllidar_a1_launch.py"),
            # Encoder Odometry (Ackermann)
            launch("encoder_odometry", "launch.py"),
        ]
    )
