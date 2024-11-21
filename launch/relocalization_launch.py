import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="visual_localization",
                namespace="visual_localization",
                executable="relocalization",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("visual_localization"),
                            "config",
                            "relocalization.yaml",
                        ]
                    )
                ],
            ),
        ]
    )
