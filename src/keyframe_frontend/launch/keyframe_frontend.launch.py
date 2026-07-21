import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def make_node(share: str, idx: int) -> Node:
    return Node(
        package="keyframe_frontend",
        executable="keyframe_frontend_node",
        name=f"keyframe_frontend_uav{idx}",
        parameters=[
            os.path.join(share, "config", "keyframe_frontend.yaml"),
            {
                "robot_id": idx,
                "odom_topic": f"/uav{idx}/fast_lio/odometry",
                "cloud_topic": f"/uav{idx}/fast_lio/cloud_registered",
                "keyframe_topic": f"/uav{idx}/consensus/keyframe",
                "submap_topic": f"/uav{idx}/consensus/submap_cloud",
                "keyframe_path_topic": f"/uav{idx}/consensus/keyframe_path",
                "use_sim_time": True,
            },
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("keyframe_frontend")
    return LaunchDescription([make_node(share, idx) for idx in (1, 2, 3)])
