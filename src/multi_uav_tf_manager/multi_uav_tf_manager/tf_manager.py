import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class TFManager(Node):
    def __init__(self):
        super().__init__("multi_uav_tf_manager")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("uav1_pose", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("uav2_pose", [10.0, 0.0, 0.0, 0.0])
        self.declare_parameter("uav3_pose", [0.0, 10.0, 0.0, 0.0])

        self.global_frame = (
            self.get_parameter("global_frame")
            .get_parameter_value()
            .string_value
        )

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        transforms = []

        for robot_name in ["uav1", "uav2", "uav3"]:
            pose = (
                self.get_parameter(f"{robot_name}_pose")
                .get_parameter_value()
                .double_array_value
            )

            if len(pose) != 4:
                raise ValueError(f"{robot_name}_pose must be [x, y, z, yaw]")

            x, y, z, yaw = pose

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.global_frame
            transform.child_frame_id = f"odom_{robot_name}"

            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(yaw / 2.0)
            transform.transform.rotation.w = math.cos(yaw / 2.0)

            transforms.append(transform)

            self.get_logger().info(
                f"Publishing static TF: {self.global_frame} -> odom_{robot_name}, "
                f"xyz=({x:.2f}, {y:.2f}, {z:.2f}), yaw={yaw:.3f}"
            )

        self.broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = TFManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
