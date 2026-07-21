#!/usr/bin/env python3
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)

Position = Tuple[float, float, float]


def lerp(a: Position, b: Position, ratio: float) -> Position:
    ratio = max(0.0, min(1.0, ratio))
    return tuple(a[i] + ratio * (b[i] - a[i]) for i in range(3))


class Uav1FastlioMission(Node):
    """三机解锁+起飞+悬停；UAV1 执行 4×4m 方形轨迹演示，UAV2/UAV3 保持悬停。
    
    切换方法（仅悬停 / 含方形轨迹）：
      在 timer_cb() 的 SETTLE 阶段末尾，修改 self.set_phase(...) 的目标阶段即可：
        self.set_phase("SQUARE")      ← 含方形轨迹演示（默认）
        self.set_phase("FINAL_HOLD")  ← 跳过方形轨迹，SETTLE 后直接永久悬停
    """

    def __init__(self) -> None:
        super().__init__("uav1_fastlio_mission")
        self.ids = (1, 2, 3)

        # ---- ROS2 QoS 配置 ----
        # 发布侧：BEST_EFFORT + TRANSIENT_LOCAL（适用于 offboard 控制指令）
        self.pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 订阅侧：BEST_EFFORT + VOLATILE（适用于传感器/状态数据流）
        self.sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        # ---- 状态存储 ----
        self.positions: Dict[int, Optional[Position]] = {i: None for i in self.ids}
        self.status: Dict[int, Optional[VehicleStatus]] = {i: None for i in self.ids}
        self.origin: Dict[int, Position] = {}

        # ---- 为每台 UAV 创建 PX4 通信接口 ----
        self.offboard_pub = {}
        self.setpoint_pub = {}
        self.command_pub = {}

        for idx in self.ids:
            prefix = f"/px4_{idx}/fmu"
            # OffboardControlMode: 告知飞控当前 offboard 模式使用的控制量类型（位置/速度/加速度等）
            self.offboard_pub[idx] = self.create_publisher(
                OffboardControlMode,
                f"{prefix}/in/offboard_control_mode",
                self.pub_qos,
            )
            # TrajectorySetpoint: 发送目标位置、速度、加速度设定点
            self.setpoint_pub[idx] = self.create_publisher(
                TrajectorySetpoint,
                f"{prefix}/in/trajectory_setpoint",
                self.pub_qos,
            )
            # VehicleCommand: 发送模式切换、解锁等飞控指令
            self.command_pub[idx] = self.create_publisher(
                VehicleCommand,
                f"{prefix}/in/vehicle_command",
                self.pub_qos,
            )
            # 订阅里程计，获取当前位置
            self.create_subscription(
                VehicleOdometry,
                f"{prefix}/out/vehicle_odometry",
                lambda msg, i=idx: self.odom_cb(i, msg),
                self.sub_qos,
            )
            # 订阅飞控状态（解锁状态、导航模式等）
            self.create_subscription(
                VehicleStatus,
                f"{prefix}/out/vehicle_status",
                lambda msg, i=idx: self.status_cb(i, msg),
                self.sub_qos,
            )

        # ---- 任务阶段状态机 ----
        # 默认流程: WAIT_ODOMETRY → PRESTREAM → ARM_OFFBOARD → TAKEOFF → SETTLE → SQUARE → FINAL_HOLD
        # 仅悬停:   将 SETTLE 末尾的 set_phase("SQUARE") 改为 set_phase("FINAL_HOLD") 即可跳过方形轨迹
        self.phase = "WAIT_ODOMETRY"
        self.phase_start = time.monotonic()
        self.last_command_time = 0.0
        self.last_status_log = 0.0

        # ---- 任务参数 ----
        self.takeoff_z = {1: -3.0, 2: -4.0, 3: -5.0}  # 各 UAV 目标高度（NED，负值=向上）
        self.prestream_sec = 3.0      # 预发送 setpoint 时间（飞控要求切换 offboard 前持续收到 setpoint）
        self.arm_timeout_sec = 15.0   # 解锁超时（超时后强制进入起飞阶段）
        self.takeoff_sec = 10.0       # 起飞爬升时间
        self.settle_sec = 5.0         # 到达目标高度后稳定悬停时间
        self.leg_sec = 8.0            # 方形轨迹每条边持续时间（秒）
        self.square_side = 4.0        # 方形轨迹边长（米）

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(
            "Mission started: three-UAV takeoff, UAV1 4x4 m square trajectory demo."
        )

    # ---- 回调函数 ----
    def odom_cb(self, idx: int, msg: VehicleOdometry) -> None:
        """接收 PX4 里程计数据，更新 UAV 当前位置（NED 坐标系）。"""
        if len(msg.position) < 3:
            return
        p = tuple(float(v) for v in msg.position[:3])
        if all(math.isfinite(v) for v in p):
            self.positions[idx] = p

    def status_cb(self, idx: int, msg: VehicleStatus) -> None:
        """接收 PX4 飞控状态（解锁状态、导航模式等）。"""
        self.status[idx] = msg

    # ---- PX4 通信辅助 ----
    def timestamp_us(self) -> int:
        """获取当前时间戳（微秒），PX4 uORB 消息必需字段。"""
        return int(self.get_clock().now().nanoseconds // 1000)

    def publish_mode(self, idx: int) -> None:
        """发送 OffboardControlMode：声明使用位置控制模式。"""
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True          # 启用位置控制
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False
        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False
        self.offboard_pub[idx].publish(msg)

    def publish_setpoint(self, idx: int, p: Position) -> None:
        """发送轨迹设定点（目标位置 NED），NaN 表示不控制速度/加速度/偏航速率。"""
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [float(p[0]), float(p[1]), float(p[2])]
        msg.velocity = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = 0.0
        msg.yawspeed = float("nan")
        self.setpoint_pub[idx].publish(msg)

    def send_command(
        self,
        idx: int,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        """发送 MAVLink 指令（如模式切换、解锁/上锁）。"""
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

    # ---- 解锁与模式切换 ----
    def request_offboard_and_arm(self) -> None:
        """每秒发送一次 OFFBOARD 模式切换 + 解锁指令（两者需同时满足才能进入 offboard 飞行）。"""
        now = time.monotonic()
        if now - self.last_command_time < 1.0:
            return
        self.last_command_time = now
        for idx in self.ids:
            # VEHICLE_CMD_DO_SET_MODE: param1=1(自定义模式), param2=6(PX4 offboard 模式)
            self.send_command(
                idx,
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
            # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1=1(解锁)
            self.send_command(
                idx,
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0,
            )
        self.get_logger().info("Sent OFFBOARD + ARM commands to all UAVs.")

    def all_armed_offboard(self) -> bool:
        """检查三台 UAV 是否均已解锁且处于 offboard 导航模式。"""
        for idx in self.ids:
            st = self.status[idx]
            if st is None:
                return False
            if st.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                return False
            if st.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                return False
        return True

    # ---- 阶段管理 ----
    def set_phase(self, phase: str) -> None:
        """切换任务阶段并记录时间戳。"""
        self.phase = phase
        self.phase_start = time.monotonic()
        self.get_logger().warn(f"MISSION PHASE -> {phase}")

    def elapsed(self) -> float:
        """当前阶段已持续时间（秒）。"""
        return time.monotonic() - self.phase_start

    # ---- 目标位置计算 ----
    def ground_targets(self) -> Dict[int, Position]:
        """地面目标位置：各 UAV 记录的原点（原地不动）。"""
        return {idx: self.origin[idx] for idx in self.ids}

    def airborne_targets(self) -> Dict[int, Position]:
        """空中悬停目标位置：xy 保持原点，z 为目标高度（NED 负值）。"""
        return {
            idx: (
                self.origin[idx][0],
                self.origin[idx][1],
                self.takeoff_z[idx],
            )
            for idx in self.ids
        }

    def takeoff_targets(self, ratio: float) -> Dict[int, Position]:
        """起飞过程中的插值目标位置（从地面到目标高度线性过渡）。"""
        result = {}
        for idx in self.ids:
            start = self.origin[idx]
            end = (start[0], start[1], self.takeoff_z[idx])
            result[idx] = lerp(start, end, ratio)
        return result

    # ---- 方形轨迹：UAV1 沿 4×4m 正方形飞行，UAV2/UAV3 保持悬停 ----
    # 如需禁用方形轨迹，在下方 SETTLE 阶段将 set_phase("SQUARE") 改为 set_phase("FINAL_HOLD")
    def square_targets(self, elapsed: float) -> Dict[int, Position]:
        """UAV1 飞 4×4m 方形轨迹（4 段折线，每段 self.leg_sec 秒），UAV2/UAV3 保持各自悬停高度。"""
        targets = self.airborne_targets()
        p0 = targets[1]
        s = self.square_side
        points = (
            p0,
            (p0[0] + s, p0[1], p0[2]),
            (p0[0] + s, p0[1] + s, p0[2]),
            (p0[0], p0[1] + s, p0[2]),
            p0,
        )
        leg = int(elapsed // self.leg_sec)
        if leg >= 4:
            targets[1] = p0
            return targets
        t = elapsed - leg * self.leg_sec
        targets[1] = lerp(points[leg], points[leg + 1], t / self.leg_sec)
        return targets

    # ---- 发布与日志 ----
    def publish_targets(self, targets: Dict[int, Position]) -> None:
        """向三台 UAV 同时发布 offboard 控制模式和目标设定点。"""
        for idx in self.ids:
            self.publish_mode(idx)
            self.publish_setpoint(idx, targets[idx])

    def log_status(self) -> None:
        """每 2 秒输出一次三台 UAV 的状态摘要（解锁状态、导航模式、当前位置）。"""
        now = time.monotonic()
        if now - self.last_status_log < 2.0:
            return
        self.last_status_log = now
        parts = []
        for idx in self.ids:
            st = self.status[idx]
            pos = self.positions[idx]
            if st is None:
                parts.append(f"UAV{idx}: status=NONE")
                continue
            pos_text = "NONE" if pos is None else (
                f"({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
            )
            parts.append(
                f"UAV{idx}: arm={st.arming_state}, nav={st.nav_state}, "
                f"preflight={getattr(st, 'pre_flight_checks_pass', 'NA')}, "
                f"pos={pos_text}"
            )
        self.get_logger().info(" | ".join(parts))

    # ---- 主循环：50Hz 定时器回调，驱动整个任务状态机 ----
    def timer_cb(self) -> None:
        self.log_status()

        # 阶段1: 等待所有 UAV 的里程计数据就绪，记录初始位置作为原点
        if self.phase == "WAIT_ODOMETRY":
            if any(self.positions[idx] is None for idx in self.ids):
                return
            self.origin = {
                idx: self.positions[idx]  # type: ignore[assignment]
                for idx in self.ids
            }
            self.get_logger().info(f"Captured local origins: {self.origin}")
            self.set_phase("PRESTREAM")

        # 阶段2: 预发送 setpoint（PX4 要求在切换 offboard 前持续收到有效 setpoint）
        if self.phase == "PRESTREAM":
            self.publish_targets(self.ground_targets())
            if self.elapsed() >= self.prestream_sec:
                self.set_phase("ARM_OFFBOARD")
            return

        # 阶段3: 发送 offboard 模式切换 + 解锁指令，等待三机全部就绪
        if self.phase == "ARM_OFFBOARD":
            self.publish_targets(self.ground_targets())
            self.request_offboard_and_arm()
            if self.all_armed_offboard():
                self.set_phase("TAKEOFF")
            elif self.elapsed() >= self.arm_timeout_sec:
                self.get_logger().warn(
                    "Arm/offboard timeout; continuing retries during TAKEOFF."
                )
                self.set_phase("TAKEOFF")
            return

        # 阶段4: 三机同步起飞到各自目标高度（线性插值过渡）
        if self.phase == "TAKEOFF":
            self.request_offboard_and_arm()
            ratio = self.elapsed() / self.takeoff_sec
            self.publish_targets(self.takeoff_targets(ratio))
            if ratio >= 1.0:
                self.set_phase("SETTLE")
            return

        # 阶段5: 到达目标高度后稳定悬停，然后进入方形轨迹演示
        if self.phase == "SETTLE":
            self.request_offboard_and_arm()
            self.publish_targets(self.airborne_targets())
            if self.elapsed() >= self.settle_sec:
                # [切换点] 如需仅悬停不飞方形轨迹，将 "SQUARE" 改为 "FINAL_HOLD"
                self.set_phase("FINAL_HOLD")
            return

        # 阶段6: UAV1 执行 4×4m 方形轨迹演示（4 段 × leg_sec 秒/段）
        if self.phase == "SQUARE":
            self.request_offboard_and_arm()
            self.publish_targets(self.square_targets(self.elapsed()))
            if self.elapsed() >= 4.0 * self.leg_sec:
                self.set_phase("FINAL_HOLD")
            return

        # 阶段7: 任务结束，三机保持悬停
        if self.phase == "FINAL_HOLD":
            self.publish_targets(self.airborne_targets())


def main(args=None) -> None:
    """任务入口：初始化 ROS2，创建节点，进入 spin 循环。"""
    rclpy.init(args=args)
    node = Uav1FastlioMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
