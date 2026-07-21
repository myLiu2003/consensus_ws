#!/usr/bin/env python3
"""Run up to three Swarm-LIO2 simulation instances on one ROS 2 host."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    vehicle_count = max(1, min(3, int(LaunchConfiguration("vehicle_count").perform(context))))
    actual_uav_num = int(LaunchConfiguration("actual_uav_num").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    config_path = PathJoinSubstitution(
        [FindPackageShare("swarm_lio"), "config", "simulation.yaml"]
    )
    actions = []
    for drone_id in range(1, vehicle_count + 1):
        node = Node(
            package="swarm_lio",
            executable="swarm_lio_node",
            name=f"swarm_lio_quad{drone_id}",
            output="screen",
            parameters=[
                config_path,
                {
                    "common/drone_id": drone_id,
                    "common/lid_topic": f"/quad{drone_id}/mid360/points",
                    "common/imu_topic": f"/quad{drone_id}/mid360/imu",
                    "multiuav/actual_uav_num": actual_uav_num,
                    "preprocess/lidar_type": 6,
                    "use_sim_time": use_sim_time,
                    "log_dir": f"/tmp/swarm_lio/quad{drone_id}",
                },
            ],
        )
        actions.append(TimerAction(period=float(drone_id - 1) * 3.0, actions=[node]))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_count", default_value="3"),
            DeclareLaunchArgument("actual_uav_num", default_value="3"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
