import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class ThreeUavWaypoints(Node):
    def __init__(self):
        super().__init__('three_uav_waypoints')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.uavs = [1, 2, 3]
        self.count = 0
        self.start_ns = self.get_clock().now().nanoseconds

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
        self.get_logger().info('three_uav_waypoints started')

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

    def arm(self, uav_id):
        self.publish_vehicle_command(
            uav_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def set_offboard_mode(self, uav_id):
        self.publish_vehicle_command(
            uav_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def waypoint(self, uav_id, t):
        routes = {
            1: [(0, 0, -3), (4, 0, -3), (4, 4, -3), (0, 4, -3), (0, 0, -3)],
            2: [(3, 0, -4), (7, 0, -4), (7, 4, -4), (3, 4, -4), (3, 0, -4)],
            3: [(0, 3, -5), (4, 3, -5), (4, 7, -5), (0, 7, -5), (0, 3, -5)],
        }
        idx = min(int(t // 8), len(routes[uav_id]) - 1)
        return routes[uav_id][idx]

    def publish_setpoint(self, uav_id, t):
        x, y, z = self.waypoint(uav_id, t)

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

        if self.count == 20:
            for i in self.uavs:
                self.set_offboard_mode(i)
                self.arm(i)
            self.get_logger().info('sent offboard + arm commands to px4_1/2/3')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ThreeUavWaypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
