from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="controls",
                executable="trajectory_follower_node",
                name="trajectory_node",
            ),
            # Node(
            #     package="controls",
            #     executable="debug_trajectory",
            #     name="debug_trajectory",
            # )
        ]
    )

