import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fast_lio")
    uav_id = LaunchConfiguration("uav_id")

    config_file = PythonExpression([
        "'", package_share, "/config/uav",
        uav_id,
        "_gazebo.yaml'"
    ])

    namespace = PythonExpression([
        "'uav", uav_id, "/fast_lio'"
    ])

    node_name = PythonExpression([
        "'fastlio_mapping_uav", uav_id, "'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "uav_id",
            default_value="1",
            description="UAV index: 1, 2, or 3",
        ),

        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name=node_name,
            namespace=namespace,
            parameters=[
                config_file,
                {"use_sim_time": True},
            ],
            remappings=[
                ("Odometry", "odometry"),
                ("Laser_map", "map"),
            ],
            output="screen",
        ),
    ])
