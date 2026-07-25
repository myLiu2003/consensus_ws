import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # 是否由本 launch 启动三机 FAST-LIO。
    # 如果外部总仿真已经启动 FAST-LIO，应保持 false，避免重复节点。
    start_fastlio = LaunchConfiguration("start_fastlio")

    loop_share = get_package_share_directory("loop_consensus")
    keyframe_share = get_package_share_directory("keyframe_frontend")
    fastlio_share = get_package_share_directory("fast_lio")

    loop_config = os.path.join(
        loop_share,
        "config",
        "loop_consensus.yaml",
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                fastlio_share,
                "launch",
                "three_uav_fastlio.launch.py",
            )
        ),
        condition=IfCondition(start_fastlio),
    )

    tf_manager_node = Node(
        package="multi_uav_tf_manager",
        executable="tf_manager",
        name="multi_uav_tf_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    keyframe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                keyframe_share,
                "launch",
                "keyframe_frontend.launch.py",
            )
        ),
        launch_arguments={
            "start_rviz": "false",
        }.items(),
    )

    loop_consensus_node = Node(
        package="loop_consensus",
        executable="loop_consensus_node",
        name="loop_consensus_node",
        output="screen",
        parameters=[loop_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_fastlio",
                default_value="false",
                description=(
                    "Start the three FAST-LIO nodes. Keep false when "
                    "FAST-LIO is already running."
                ),
            ),
            fastlio_launch,
            tf_manager_node,
            keyframe_launch,
            loop_consensus_node,
        ]
    )
