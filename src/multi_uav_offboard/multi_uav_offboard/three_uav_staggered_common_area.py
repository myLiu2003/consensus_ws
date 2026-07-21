import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class ThreeUavStaggeredCommonArea(Node):
    def __init__(self):
        super().__init__('three_uav_staggered_common_area')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.uavs = [1, 2, 3]
        self.start_delay = {1: 0.0, 2: 5.0, 3: 10.0}
        self.altitude = {1: -3.0, 2: -4.0, 3: -5.0}
        self.start_xy = {1: (0.0, 0.0), 2: (3.0, 0.0), 3: (0.0, 3.0)}
        self.common_xy = {1: (3.5, 3.5), 2: (4.5, 3.5), 3: (4.0, 4.5)}

        self.start_ns = self.get_clock().now().nanoseconds
        self.armed = set()
        self.count = 0

        self.mode_pubs = {}
        self.setpoint_pubs = {}
        self.command_pubs = {}

        for i in self.uavs:
            ns = f'/px4_{i}'
            self.mode_pubs[i] = self.create_publisher(
                OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos)
            self.setpoint_pubs[i] = self.create_publisher(
                TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos)
            self.command_pubs[i] = self.create_publisher(
                VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('staggered common-area mission started')
        self.get_logger().info('common area: x=[3.5,4.5], y=[3.5,4.5], z=[-5,-3]')

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_vehicle_command(self, uav_id, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = uav_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pubs[uav_id].publish(msg)

    def set_offboard_mode(self, uav_id):
        self.publish_vehicle_command(
            uav_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def arm(self, uav_id):
        self.publish_vehicle_command(
            uav_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def setpoint_for(self, uav_id, t):
        delay = self.start_delay[uav_id]
        sx, sy = self.start_xy[uav_id]
        cx, cy = self.common_xy[uav_id]
        z = self.altitude[uav_id]

        if t < delay:
            return sx, sy, 0.0
        if t < delay + 8.0:
            return sx, sy, z
        if t < delay + 18.0:
            alpha = (t - delay - 8.0) / 10.0
            return sx + alpha * (cx - sx), sy + alpha * (cy - sy), z
        return cx, cy, z

    def publish_setpoint(self, uav_id, t):
        x, y, z = self.setpoint_for(uav_id, t)

        mode = OffboardControlMode()
        mode.timestamp = self.timestamp()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        if hasattr(mode, 'actuator'):
            mode.actuator = False
        if hasattr(mode, 'thrust_and_torque'):
            mode.thrust_and_torque = False
        if hasattr(mode, 'direct_actuator'):
            mode.direct_actuator = False
        self.mode_pubs[uav_id].publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = 0.0
        self.setpoint_pubs[uav_id].publish(sp)

    def timer_callback(self):
        t = (self.get_clock().now().nanoseconds - self.start_ns) / 1e9

        for i in self.uavs:
            self.publish_setpoint(i, t)

        for i in self.uavs:
            if i not in self.armed and t >= self.start_delay[i] + 2.0:
                self.set_offboard_mode(i)
                self.arm(i)
                self.armed.add(i)
                self.get_logger().info(f'px4_{i}: offboard+arm at t={t:.1f}s')

        if self.count % 50 == 0:
            self.get_logger().info(f't={t:.1f}s running staggered common-area mission')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ThreeUavStaggeredCommonArea()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
