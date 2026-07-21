"""
Swarm-LIO2 ROS2 launch file
Launches: imu_publisher (external IMU) + livox_ros_driver2 (MID360) + swarm_lio_node + udp_online_node

Usage:
  ros2 launch swarm_lio swarm_lio2.launch.py drone_id:=1 namespace:=dp550_3
  ros2 launch swarm_lio swarm_lio2.launch.py config_file:=mid360.yaml

Parameters are loaded from config/<config_file> (default: mid360.yaml).
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- arguments ---
    drone_id = LaunchConfiguration("drone_id")
    ns = LaunchConfiguration("namespace")
    config_file = LaunchConfiguration("config_file", default="mid360.yaml")
    lid_topic = LaunchConfiguration("lid_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    actual_uav_num = LaunchConfiguration("actual_uav_num")
    sensor_mode = LaunchConfiguration("sensor_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_sensor_drivers = LaunchConfiguration("start_sensor_drivers")
    start_udp_bridge = LaunchConfiguration("start_udp_bridge")
    log_dir = LaunchConfiguration("log_dir")

    declare_drone_id = DeclareLaunchArgument("drone_id", default_value="1")
    declare_ns = DeclareLaunchArgument("namespace", default_value="")
    declare_config = DeclareLaunchArgument("config_file", default_value="mid360.yaml")

    # --- resolve config path ---
    config_path = PathJoinSubstitution([
        FindPackageShare("swarm_lio"),
        "config",
        config_file,
    ])

    # 1. External IMU (e.g., ICM20948 over I2C)
    imu_node = Node(
        package="imu_publisher",
        executable="imu_publisher_node",
        namespace=ns,
        name="imu_publisher",
        output="screen",
        condition=IfCondition(start_sensor_drivers),
    )

    # 2. Livox MID360 driver
    mid360_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        namespace=ns,
        name="livox_lidar_publisher",
        output="screen",
        parameters=[{
            "xfer_format": 0,            # CustomMsg
            "multi_topic": 0,
            "bd_list": "00:00:00:00:00:00",
        }],
        condition=IfCondition(start_sensor_drivers),
    )

    # 3. Swarm-LIO2 state estimation core
    swarm_lio_node = Node(
        package="swarm_lio",
        executable="swarm_lio_node",
        namespace=ns,
        name="swarm_lio",
        output="screen",
        parameters=[
            config_path,
            {
                "common/drone_id": ParameterValue(drone_id, value_type=int),
                "common/lid_topic": lid_topic,
                "common/imu_topic": imu_topic,
                "multiuav/actual_uav_num": ParameterValue(actual_uav_num, value_type=int),
                "preprocess/lidar_type": ParameterValue(sensor_mode, value_type=int),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "log_dir": log_dir,
            },
        ],
    )

    # 4. UDP bridge for multi-drone communication
    udp_node = Node(
        package="udp_bridge",
        executable="udp_online_node",
        namespace=ns,
        name="udp_bridge",
        output="screen",
        parameters=[{
            "drone_id": ParameterValue(drone_id, value_type=int),
            "bind_ip": LaunchConfiguration("bind_ip"),
            "broadcast_ip": LaunchConfiguration("broadcast_ip"),
            "port": ParameterValue(LaunchConfiguration("udp_port"), value_type=int),
            "log_dir": log_dir,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(start_udp_bridge),
    )

    return LaunchDescription([
        declare_drone_id,
        declare_ns,
        declare_config,
        DeclareLaunchArgument("lid_topic", default_value="livox/lidar"),
        DeclareLaunchArgument("imu_topic", default_value="livox/imu"),
        DeclareLaunchArgument("actual_uav_num", default_value="3"),
        DeclareLaunchArgument("sensor_mode", default_value="1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_sensor_drivers", default_value="true"),
        DeclareLaunchArgument("start_udp_bridge", default_value="true"),
        DeclareLaunchArgument("bind_ip", default_value=""),
        DeclareLaunchArgument("broadcast_ip", default_value="255.255.255.255"),
        DeclareLaunchArgument("udp_port", default_value="8821"),
        DeclareLaunchArgument("log_dir", default_value="/tmp/swarm_lio"),
        LogInfo(msg=["[Swarm-LIO2] Starting drone ", drone_id, " config=", config_file]),
        # staggered start to avoid race conditions
        TimerAction(period=1.0, actions=[imu_node]),
        TimerAction(period=2.0, actions=[mid360_node]),
        TimerAction(period=5.0, actions=[swarm_lio_node]),
        TimerAction(period=6.0, actions=[udp_node]),
    ])
