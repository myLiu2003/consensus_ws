#!/usr/bin/env python3
"""Run three isolated UDP bridges on one host using loopback IPs and ROS domains."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    base_domain = int(LaunchConfiguration("base_domain_id").perform(context))
    port = int(LaunchConfiguration("port").perform(context))
    ips = ["127.0.0.11", "127.0.0.12", "127.0.0.13"]
    actions = []
    for index, bind_ip in enumerate(ips):
        drone_id = index + 1
        actions.append(
            TimerAction(
                period=float(index),
                actions=[
                    Node(
                        package="udp_bridge",
                        executable="udp_online_node",
                        name=f"udp_bridge_q{drone_id}",
                        output="screen",
                        additional_env={"ROS_DOMAIN_ID": str(base_domain + index)},
                        parameters=[{
                            "drone_id": drone_id,
                            "bind_ip": bind_ip,
                            "broadcast_ip": "127.255.255.255",
                            "peer_ips": [ip for ip in ips if ip != bind_ip],
                            "port": port,
                            "log_dir": f"/tmp/swarm_lio/udp_q{drone_id}",
                            "use_sim_time": False,
                        }],
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_domain_id", default_value="71"),
        DeclareLaunchArgument("port", default_value="8821"),
        OpaqueFunction(function=_setup),
    ])
