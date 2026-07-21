import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start_gazebo(context):
    package_share = get_package_share_directory("student_center_sim")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(
        package_share,
        "worlds",
        "student_center.sdf",
    )

    headless_text = (
        LaunchConfiguration("headless")
        .perform(context)
        .strip()
        .lower()
    )

    headless = headless_text in {
        "1",
        "true",
        "yes",
        "on",
    }

    server_only = "-s " if headless else ""
    gz_args = f"-r -v 4 {server_only}{world_path}"

    gz_launch = os.path.join(
        ros_gz_share,
        "launch",
        "gz_sim.launch.py",
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                "gz_args": gz_args,
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("student_center_sim")
    home = os.path.expanduser("~")
    px4_root = os.path.join(home, "PX4-Autopilot")

    resource_paths = [
        os.path.join(package_share, "models"),
        os.path.join(package_share, "worlds"),
        os.path.join(px4_root, "Tools", "simulation", "gz", "models"),
        os.path.join(px4_root, "Tools", "simulation", "gz", "worlds"),
    ]

    existing_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_path:
        resource_paths.append(existing_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="true: Gazebo server only; false: start Gazebo GUI",
        ),

        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(resource_paths),
        ),

        OpaqueFunction(function=start_gazebo),
    ])
