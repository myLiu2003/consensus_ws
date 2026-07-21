#!/usr/bin/env python3

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)


Position = Tuple[float, float, float]


class Uav1FastLioMotionTest(Node):
    """Move UAV1 along a square while UAV2 and UAV3 hold position."""

    def __init__(self) -> None:
        super().__init__("uav1_fastlio_motion_test")

        self.uav_ids = (1, 2, 3)

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.positions: Dict[int, Optional[Position]] = {
            idx: None for idx in self.uav_ids
        }
        self.hold_positions: Dict[int, Position] = {}

        self.offboard_publishers = {}
        self.setpoint_publishers = {}
        self.subscriptions = []

        for idx in self.uav_ids:
            prefix = f"/px4_{idx}/fmu"

            self.offboard_publishers[idx] = self.create_publisher(
                OffboardControlMode,
                f"{prefix}/in/offboard_control_mode",
                self.qos,
            )

            self.setpoint_publishers[idx] = self.create_publisher(
                TrajectorySetpoint,
                f"{prefix}/in/trajectory_setpoint",
                self.qos,
            )

            self.subscriptions.append(
                self.create_subscription(
                    VehicleOdometry,
                    f"{prefix}/out/vehicle_odometry",
                    lambda msg, uav_id=idx: self.odometry_callback(
                        uav_id, msg
                    ),
                    self.qos,
                )
            )

        self.reference_ready = False
        self.sequence_start_wall: Optional[float] = None
        self.last_phase = ""

        # 为手动关闭旧控制器预留时间。
        self.handover_hold_sec = 12.0

        # 4 m / 0.5 m/s = 8 s。
        self.segment_duration_sec = 8.0
        self.final_hold_sec = 5.0

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "UAV1 FAST-LIO motion test node started. "
            "Waiting for three vehicle odometry streams."
        )

    def odometry_callback(
        self,
        uav_id: int,
        msg: VehicleOdometry,
    ) -> None:
        if len(msg.position) < 3:
            return

        position = tuple(float(v) for v in msg.position[:3])

        if not all(math.isfinite(v) for v in position):
            return

        self.positions[uav_id] = position

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def publish_offboard_mode(self, uav_id: int) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_publishers[uav_id].publish(msg)

    def publish_position(
        self,
        uav_id: int,
        position: Position,
    ) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [
            float(position[0]),
            float(position[1]),
            float(position[2]),
        ]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.jerk = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        msg.yawspeed = 0.0

        self.setpoint_publishers[uav_id].publish(msg)

    @staticmethod
    def interpolate(
        start: Position,
        end: Position,
        ratio: float,
    ) -> Position:
        ratio = max(0.0, min(1.0, ratio))

        return tuple(
            start[i] + ratio * (end[i] - start[i])
            for i in range(3)
        )

    def initialize_reference(self) -> bool:
        if self.reference_ready:
            return True

        if any(self.positions[idx] is None for idx in self.uav_ids):
            return False

        self.hold_positions = {
            idx: self.positions[idx]  # type: ignore[assignment]
            for idx in self.uav_ids
        }

        self.reference_ready = True
        self.sequence_start_wall = time.monotonic()

        p = self.hold_positions[1]

        self.get_logger().info(
            "All odometry streams received."
        )
        self.get_logger().info(
            f"UAV1 reference NED = "
            f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
        )
        self.get_logger().warn(
            "HANDOVER WINDOW STARTED: stop the old "
            "three_uav_waypoints node now. "
            "UAV1 motion begins after 12 seconds."
        )

        return True

    def uav1_target(
        self,
        elapsed: float,
    ) -> Tuple[Position, str]:
        p0 = self.hold_positions[1]

        p1 = (p0[0] + 4.0, p0[1], p0[2])
        p2 = (p0[0] + 4.0, p0[1] + 4.0, p0[2])
        p3 = (p0[0], p0[1] + 4.0, p0[2])
        p4 = p0

        if elapsed < self.handover_hold_sec:
            return p0, "handover_hold"

        t = elapsed - self.handover_hold_sec
        d = self.segment_duration_sec

        segments = [
            (p0, p1, "leg_1_positive_x"),
            (p1, p2, "leg_2_positive_y"),
            (p2, p3, "leg_3_negative_x"),
            (p3, p4, "leg_4_negative_y"),
        ]

        for start, end, phase in segments:
            if t < d:
                return self.interpolate(start, end, t / d), phase
            t -= d

        return p4, "final_hold"

    def timer_callback(self) -> None:
        if not self.initialize_reference():
            return

        assert self.sequence_start_wall is not None

        elapsed = time.monotonic() - self.sequence_start_wall
        uav1_target, phase = self.uav1_target(elapsed)

        if phase != self.last_phase:
            self.last_phase = phase
            self.get_logger().info(f"Motion phase: {phase}")

        # 三架飞机都持续接收Offboard心跳。
        for idx in self.uav_ids:
            self.publish_offboard_mode(idx)

        self.publish_position(1, uav1_target)
        self.publish_position(2, self.hold_positions[2])
        self.publish_position(3, self.hold_positions[3])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Uav1FastLioMotionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
