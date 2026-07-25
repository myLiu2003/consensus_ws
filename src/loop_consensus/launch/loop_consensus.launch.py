import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("loop_consensus"),
        "config",
        "loop_consensus.yaml",
    )

    loop_consensus_node = Node(
        package="loop_consensus",
        executable="loop_consensus_node",
        name="loop_consensus_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        loop_consensus_node,
    ])