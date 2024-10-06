
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="serial_com",
            #     executable="arduino_read",
            #     name="arduino_read",
            #     parameters=[]
            # ),
            # Node(
            #     package="serial_com",
            #     executable="arduino_write",
            #     name="arduino_write",
            #     parameters=[]
            # )
            Node(
                package="serial_com",
                executable="arduino_com",
                name="arduino_com",
                parameters=[]
            )
        ]
    )

