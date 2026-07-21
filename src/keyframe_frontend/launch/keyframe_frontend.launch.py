import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
                "keyframe_marker_topic": f"/uav{idx}/consensus/keyframe_markers",
                "use_sim_time": True,
            },
        ],
        output="screen",
    )


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("keyframe_frontend")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    actions = [
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Whether to start RViz with the keyframe visualization config.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(share, "rviz", "keyframe_frontend_uav1.rviz"),
            description="RViz config used when start_rviz is true.",
        ),
    ]

    actions.extend([make_node(share, idx) for idx in (1, 2, 3)])
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="keyframe_frontend_rviz",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(start_rviz),
        )
    )

    return LaunchDescription(actions)
