from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to the simulator bringup launch file
    teleop_launch_path = os.path.join(
        get_package_share_directory("teleop"),
        "launch",
        "launch.py",
    )

    autobrake_launch_path = os.path.join(
        get_package_share_directory("controls"),
        "launch",
        "autobrake_launch.py"
    )

    serial_com_launch_path = os.path.join(
        get_package_share_directory("serial_com"),
        "launch",
        "launch.py"
    )

    rplidar_launch_path = os.path.join(
        get_package_share_directory("sllidar_ros2"),
        "launch",
        "sllidar_a1_launch.py"
    )

    encoder_odometry_launch_path = os.path.join(
        get_package_share_directory("encoder_odometry"),
        "launch",
        "launch.py"
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
                    '0.035', '0.04', '0',  # Translation: x = 0.035, y = 0.04, z = 0 (meters)
                    '0', '0', '1', '0', # Rotation: M_PI
                    'base_link',
                    'laser_frame'
                ]
            ),

            # Run joystick reader
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[] #ros2 uses events, so don't try and direct this to /dev
            ),
            # Launch RPLidar A1
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rplidar_launch_path),
                launch_arguments={}.items(),
            ),
            # Launch teleop
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch_path),
                launch_arguments={}.items(),
            ),
            # Launch autobrake
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(autobrake_launch_path),
                launch_arguments={}.items(),
            ),
            # Launch serial communication
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(serial_com_launch_path),
                launch_arguments={}.items(),
            ),
            # Launch encoder odometry
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(encoder_odometry_launch_path),
                launch_arguments={}.items(),
            ),
        ]
    )
