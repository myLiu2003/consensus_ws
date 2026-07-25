from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("loop_consensus"),
        "config",
        "loop_consensus.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="loop_consensus",
                executable="loop_consensus_node",
                name="loop_consensus_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
