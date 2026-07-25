#!/usr/bin/env python3
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

Position = Tuple[float, float, float]
WorldPoint = Tuple[float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ThreeUavOverlapMission(Node):
    """Fly all UAVs through the same closed loop with staggered entry times."""

    def __init__(self) -> None:
        super().__init__('three_uav_overlap_mission')
        self.ids = (1, 2, 3)
        self.altitude_m = float(
            self.declare_parameter('altitude_m', 3.0).value)
        self.speed_mps = float(
            self.declare_parameter('speed_mps', 0.8).value)
        self.entry_delay_sec = {
            1: float(self.declare_parameter(
                'uav1_entry_delay_sec', 0.0).value),
            2: float(self.declare_parameter(
                'uav2_entry_delay_sec', 12.0).value),
            3: float(self.declare_parameter(
                'uav3_entry_delay_sec', 24.0).value),
        }

        self.spawn_enu: Dict[int, WorldPoint] = {
            1: (-6.0, -12.0),
            2: (6.0, -12.0),
            3: (0.0, -12.0),
        }
        shared_loop: List[WorldPoint] = [
            (0.0, -10.0),
            (0.0, -5.0),
            (0.0, 0.0),
            (-4.0, 5.0),
            (0.0, 10.0),
            (4.0, 5.0),
            (0.0, 0.0),
            (4.0, -5.0),
            (0.0, -10.0),
        ]
        approaches: Dict[int, List[WorldPoint]] = {
            1: [(-6.0, -12.0), (-3.0, -11.0), (0.0, -10.0)],
            2: [(6.0, -12.0), (3.0, -11.0), (0.0, -10.0)],
            3: [(0.0, -12.0), (0.0, -10.0)],
        }
        # Two passes provide both intra-robot loops and cross-UAV overlap.
        self.routes = {
            idx: approaches[idx] + shared_loop[1:] + shared_loop[1:]
            for idx in self.ids
        }

        self.pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.positions: Dict[int, Optional[Position]] = {
            idx: None for idx in self.ids
        }
        self.status: Dict[int, Optional[VehicleStatus]] = {
            idx: None for idx in self.ids
        }
        self.origin: Dict[int, Position] = {}
        self.offboard_pub = {}
        self.setpoint_pub = {}
        self.command_pub = {}

        for idx in self.ids:
            prefix = f'/px4_{idx}/fmu'
            self.offboard_pub[idx] = self.create_publisher(
                OffboardControlMode,
                f'{prefix}/in/offboard_control_mode',
                self.pub_qos,
            )
            self.setpoint_pub[idx] = self.create_publisher(
                TrajectorySetpoint,
                f'{prefix}/in/trajectory_setpoint',
                self.pub_qos,
            )
            self.command_pub[idx] = self.create_publisher(
                VehicleCommand,
                f'{prefix}/in/vehicle_command',
                self.pub_qos,
            )
            self.create_subscription(
                VehicleOdometry,
                f'{prefix}/out/vehicle_odometry',
                lambda msg, uav_id=idx: self.odom_callback(uav_id, msg),
                self.sub_qos,
            )
            self.create_subscription(
                VehicleStatus,
                f'{prefix}/out/vehicle_status_v1',
                lambda msg, uav_id=idx: self.status_callback(uav_id, msg),
                self.sub_qos,
            )

        self.phase = 'WAIT_ODOMETRY'
        self.phase_start = time.monotonic()
        self.last_command_time = 0.0
        self.last_log_time = 0.0
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            'Three-UAV overlap mission ready: altitude=%.1fm speed=%.1fm/s '
            'entry_delays=%s' % (
                self.altitude_m,
                self.speed_mps,
                self.entry_delay_sec,
            )
        )

    def odom_callback(self, idx: int, msg: VehicleOdometry) -> None:
        position = tuple(float(value) for value in msg.position[:3])
        if len(position) == 3 and all(
                math.isfinite(value) for value in position):
            self.positions[idx] = position  # type: ignore[assignment]

    def status_callback(self, idx: int, msg: VehicleStatus) -> None:
        self.status[idx] = msg

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def publish_mode(self, idx: int) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = False
        self.offboard_pub[idx].publish(msg)

    def publish_setpoint(
            self, idx: int, position: Position, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [float(value) for value in position]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float(yaw)
        msg.yawspeed = float('nan')
        self.setpoint_pub[idx].publish(msg)

    def send_command(
        self,
        idx: int,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 0
        msg.target_component = 0
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub[idx].publish(msg)

    def request_offboard_and_arm(self) -> None:
        now = time.monotonic()
        if now - self.last_command_time < 1.0:
            return
        self.last_command_time = now
        for idx in self.ids:
            self.send_command(
                idx,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
            self.send_command(
                idx,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )

    def all_armed_offboard(self) -> bool:
        return all(
            self.status[idx] is not None
            and self.status[idx].arming_state
            == VehicleStatus.ARMING_STATE_ARMED
            and self.status[idx].nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            for idx in self.ids
        )

    def set_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_start = time.monotonic()
        self.get_logger().warn(f'MISSION PHASE -> {phase}')

    def elapsed(self) -> float:
        return time.monotonic() - self.phase_start

    def hover_target(self, idx: int, ratio: float = 1.0) -> Position:
        origin = self.origin[idx]
        target_z = origin[2] - self.altitude_m * clamp(ratio, 0.0, 1.0)
        return origin[0], origin[1], target_z

    def world_to_local_ned(self, idx: int, point: WorldPoint) -> Position:
        east, north = point
        spawn_east, spawn_north = self.spawn_enu[idx]
        origin = self.origin[idx]
        return (
            origin[0] + north - spawn_north,
            origin[1] + east - spawn_east,
            origin[2] - self.altitude_m,
        )

    def route_target(
            self, idx: int, route_time: float
    ) -> Tuple[Position, float, bool]:
        points = self.routes[idx]
        if route_time <= 0.0:
            return self.hover_target(idx), 0.0, False

        remaining = route_time
        for start, end in zip(points, points[1:]):
            delta_east = end[0] - start[0]
            delta_north = end[1] - start[1]
            distance = math.hypot(delta_east, delta_north)
            duration = distance / max(self.speed_mps, 0.1)
            yaw_enu = math.atan2(delta_north, delta_east)
            yaw_ned = normalize_angle(math.pi / 2.0 - yaw_enu)
            if remaining <= duration:
                ratio = clamp(remaining / duration, 0.0, 1.0)
                interpolated = (
                    start[0] + ratio * delta_east,
                    start[1] + ratio * delta_north,
                )
                target = self.world_to_local_ned(idx, interpolated)
                return target, yaw_ned, False
            remaining -= duration

        return self.world_to_local_ned(idx, points[-1]), 0.0, True

    def publish_targets(
            self, targets: Dict[int, Tuple[Position, float]]) -> None:
        for idx in self.ids:
            self.publish_mode(idx)
            self.publish_setpoint(idx, targets[idx][0], targets[idx][1])

    def log_status(self) -> None:
        now = time.monotonic()
        if now - self.last_log_time < 2.0:
            return
        self.last_log_time = now
        summary = []
        for idx in self.ids:
            status = self.status[idx]
            position = self.positions[idx]
            summary.append(
                f'UAV{idx}:status={"OK" if status else "NONE"},pos={position}'
            )
        self.get_logger().info(f'phase={self.phase} | ' + ' | '.join(summary))

    def timer_callback(self) -> None:
        self.log_status()
        if self.phase == 'WAIT_ODOMETRY':
            if any(self.positions[idx] is None for idx in self.ids):
                return
            self.origin = {
                idx: self.positions[idx]  # type: ignore[assignment]
                for idx in self.ids
            }
            self.set_phase('PRESTREAM')

        if self.phase == 'PRESTREAM':
            self.publish_targets({
                idx: (self.origin[idx], 0.0) for idx in self.ids
            })
            if self.elapsed() >= 3.0:
                self.set_phase('ARM_OFFBOARD')
            return

        if self.phase == 'ARM_OFFBOARD':
            self.publish_targets({
                idx: (self.origin[idx], 0.0) for idx in self.ids
            })
            self.request_offboard_and_arm()
            if self.all_armed_offboard():
                self.set_phase('TAKEOFF')
            return

        if self.phase == 'TAKEOFF':
            self.request_offboard_and_arm()
            ratio = self.elapsed() / 8.0
            self.publish_targets({
                idx: (self.hover_target(idx, ratio), 0.0)
                for idx in self.ids
            })
            if ratio >= 1.0:
                self.set_phase('SETTLE')
            return

        if self.phase == 'SETTLE':
            self.publish_targets({
                idx: (self.hover_target(idx), 0.0)
                for idx in self.ids
            })
            if self.elapsed() >= 5.0:
                self.set_phase('ROUTE')
            return

        if self.phase == 'ROUTE':
            targets = {}
            all_finished = True
            for idx in self.ids:
                target, yaw, finished = self.route_target(
                    idx,
                    self.elapsed() - self.entry_delay_sec[idx],
                )
                targets[idx] = (target, yaw)
                all_finished = all_finished and finished
            self.publish_targets(targets)
            if all_finished:
                self.set_phase('FINAL_HOLD')
            return

        if self.phase == 'FINAL_HOLD':
            self.publish_targets({
                idx: (self.world_to_local_ned(idx, self.routes[idx][-1]), 0.0)
                for idx in self.ids
            })


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThreeUavOverlapMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
