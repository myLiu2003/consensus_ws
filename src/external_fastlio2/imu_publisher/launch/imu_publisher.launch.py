from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="dp550_2"),
        DeclareLaunchArgument("i2c_bus", default_value="/dev/i2c-8"),
        DeclareLaunchArgument("i2c_addr", default_value="105"),
        DeclareLaunchArgument("frame_id", default_value="imu_link"),
        DeclareLaunchArgument("publish_topic", default_value="imu/data_raw"),
        DeclareLaunchArgument("publish_rate_hz", default_value="300.0"),
        DeclareLaunchArgument("accel_covariance", default_value="0.04"),
        DeclareLaunchArgument("gyro_covariance", default_value="0.02"),
        PushRosNamespace(namespace),
        Node(
            package="imu_publisher",
            executable="imu_publisher_node",
            name="imu_publisher",
            output="screen",
            parameters=[{
                "i2c_bus": LaunchConfiguration("i2c_bus"),
                "i2c_addr": ParameterValue(LaunchConfiguration("i2c_addr"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_topic": LaunchConfiguration("publish_topic"),
                "publish_rate_hz": ParameterValue(LaunchConfiguration("publish_rate_hz"), value_type=float),
                "accel_covariance": ParameterValue(LaunchConfiguration("accel_covariance"), value_type=float),
                "gyro_covariance": ParameterValue(LaunchConfiguration("gyro_covariance"), value_type=float),
            }],
        ),
    ])
