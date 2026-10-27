#!/usr/bin/env python3
"""Feed the drone's position to ros2tap's geofence.

    source px4-env.sh && python3 flight/pose_feed.py [--file /tmp/ros2tap-drone-pose.json]

PX4's position (/fmu/out/vehicle_local_position_v1) is published by the uXRCE-DDS agent
straight through Fast DDS, never through rmw_publish, so ros2tap's probe does not see it.
This node subscribes to it and keeps a small JSON file {"x", "y", "z"} current (PX4 local
NED frame: x north, y east, z down, metres from home), written and renamed 20 times a
second. ros2tap's `pose_file` geofences read it and ignore it when older than a second.
"""
import argparse
import json
import os
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PERIOD = 0.05


class PoseFeed(Node):
    def __init__(self, path: str) -> None:
        super().__init__("pose_feed")
        self.path = path
        self.tmp = path + ".tmp"
        self.last_write = 0.0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_position, qos)
        self.get_logger().info(f"writing the drone position to {path}")

    def on_position(self, msg: VehicleLocalPosition) -> None:
        now = time.monotonic()
        if now - self.last_write < PERIOD or not (msg.xy_valid and msg.z_valid):
            return
        self.last_write = now
        with open(self.tmp, "w") as f:
            json.dump({"x": msg.x, "y": msg.y, "z": msg.z}, f)
        os.replace(self.tmp, self.path)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--file", default="/tmp/ros2tap-drone-pose.json")
    args = ap.parse_args()
    rclpy.init()
    node = PoseFeed(args.file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
