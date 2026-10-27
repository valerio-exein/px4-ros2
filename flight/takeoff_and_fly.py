#!/usr/bin/env python3
"""Offboard flight from ROS 2: take off, fly a loop around the home point, return and land.

    source px4-env.sh && python3 flight/takeoff_and_fly.py [--alt 60] [--radius 150] [--points 12]

Based on px4_ros_com's offboard_control.py, with the topic names of this PX4 build
(vehicle_local_position_v1, vehicle_status_v4) and a waypoint route. Positions are PX4's
local NED frame in metres from home: x north, y east, z down (so altitude A is z = -A).
With home at the Colosseum (see run-px4.sh) the loop circles it.
"""
import argparse
import math

import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

REACHED_M = 3.0  # distance at which a waypoint counts as reached


class TakeoffAndFly(Node):
    def __init__(self, alt: float, radius: float, points: int) -> None:
        super().__init__('takeoff_and_fly')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_position, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.on_status, qos)

        self.position = None
        self.status = None
        z = -alt
        # Climb straight up, fly the loop, come back over home.
        loop = [(radius * math.cos(2 * math.pi * i / points), radius * math.sin(2 * math.pi * i / points), z)
                for i in range(points + 1)]
        self.route = [(0.0, 0.0, z)] + loop + [(0.0, 0.0, z)]
        self.target = 0
        self.ticks = 0
        self.landing = False
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'route: climb to {alt:.0f} m, loop of radius {radius:.0f} m '
                               f'({points} legs), return, land')

    def on_position(self, msg: VehicleLocalPosition) -> None:
        self.position = msg

    def on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def tick(self) -> None:
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        if self.landing:
            # PX4 lands and disarms by itself; finish once it has.
            if self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('landed and disarmed, done')
                raise SystemExit
            return

        # Offboard needs a steady heartbeat and setpoint stream (> 2 Hz) before and during the flight.
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = now_us
        self.mode_pub.publish(mode)

        x, y, z = self.route[self.target]
        sp = TrajectorySetpoint()
        sp.position = [x, y, z]
        sp.yaw = self.heading_to(x, y)
        sp.timestamp = now_us
        self.setpoint_pub.publish(sp)

        self.ticks += 1
        if self.ticks == 10:  # one second of setpoints first, then switch mode and arm
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 6 = offboard
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('offboard mode + arm requested')
        if self.position is None or not self.position.xy_valid:
            return

        if math.dist((self.position.x, self.position.y, self.position.z), (x, y, z)) < REACHED_M:
            self.target += 1
            if self.target == len(self.route):
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing = True
                self.get_logger().info('route done, landing')
            else:
                nx, ny, _ = self.route[self.target]
                self.get_logger().info(f'waypoint {self.target}/{len(self.route) - 1}: '
                                       f'N {nx:+.0f} m, E {ny:+.0f} m, alt {-z:.0f} m')

    def heading_to(self, x: float, y: float) -> float:
        """Face the direction of travel (yaw from north, NED)."""
        if self.position is None or math.hypot(x - self.position.x, y - self.position.y) < 5.0:
            return float('nan')  # NaN = keep current heading
        return math.atan2(y - self.position.y, x - self.position.x)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--alt', type=float, default=60.0, help='flight altitude above home, m')
    ap.add_argument('--radius', type=float, default=150.0, help='loop radius around home, m')
    ap.add_argument('--points', type=int, default=12, help='waypoints on the loop')
    args = ap.parse_args()
    rclpy.init()
    node = TakeoffAndFly(args.alt, args.radius, args.points)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
