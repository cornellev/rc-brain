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

    return LaunchDescription(
        [
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
        ]
    )
