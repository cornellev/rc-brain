import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_path(package, dir, file):
    return os.path.join(get_package_share_directory(package), dir, file)


def launch(package, file, launch_folder="launch", arguments={}):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_path(package, launch_folder, file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("vision"), "config", "occupancy.yaml"
    )

    return LaunchDescription(
        [
            launch(
                "zed_wrapper",
                "zed_camera.launch.py",
                arguments={
                    "camera_model": "zed2",
                    "publish_urdf": "false",
                    "publish_tf": "false",
                    "publish_map_tf": "false",
                    "publish_imu_tf": "false",
                },
            ),
            Node(
                package="tf2_ros",
                exec_name="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "zed_camera_frame",
                ],
            ),
            Node(
                package="vision",
                exec_name="occupancy_transformer",
                name="occupancy_transformer",
                parameters=[config],
            ),
        ]
    )