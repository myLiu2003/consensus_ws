import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def make_node(share, idx):
    return Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name=f"fastlio_mapping_uav{idx}",
        namespace=f"uav{idx}/fast_lio",
        parameters=[
            os.path.join(share, "config", f"uav{idx}_gazebo.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("Odometry", "odometry"), ("Laser_map", "map")],
        output="screen",
    )

def generate_launch_description():
    share = get_package_share_directory("fast_lio")
    return LaunchDescription([make_node(share, idx) for idx in (1, 2, 3)])
