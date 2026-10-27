#!/usr/bin/env python3
"""Simulated downward-looking drone camera over real aerial imagery of Rome (ROS 2).

    source px4-env.sh
    python3 camera/aerial_camera.py [--fov 90] [--rate 5] [--heading-up] [--snapshot /path/frame.jpg]

Reads the drone's position from PX4 (/fmu/out/vehicle_global_position for lat/lon,
/fmu/out/vehicle_local_position_v1 for height above home and heading), cuts the patch of ground
a nadir camera with the given field of view would see out of camera/ortho/ortho.jpg (see
fetch_ortho.py), north up (or the drone's heading up, --heading-up), and publishes
    /drone/camera/image_raw     sensor_msgs/Image (bgr8, 640x480)
    /drone/camera/camera_info   sensor_msgs/CameraInfo
A downward-looking camera only: no perspective, no buildings in 3D. Image credit (CC BY 4.0) is
drawn on every frame, as the licence requires.

Sign injection: while /tmp/drone-sign.json exists (camera/sign.py place|remove), the marking
it names is drawn on the ground at its lat/lon, in the same geometry as the imagery.
"""
import argparse
import json
import math
import os
import pathlib
import sys

import cv2
import numpy as np
import rclpy
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # geo.py, also under python3 -I
from geo import to_utm33  # noqa: E402

WIDTH, HEIGHT = 640, 480
MIN_HEIGHT_M = 1.5        # camera height used on the ground
SIGN_CONTROL = "/tmp/drone-sign.json"   # camera/sign.py
NO_IMAGERY = (60, 60, 60)  # outside the downloaded area


class AerialCamera(Node):
    def __init__(self, fov_deg: float, rate: float, snapshot: str | None, heading_up: bool = False) -> None:
        super().__init__("aerial_camera")
        meta = json.loads((HERE / "ortho" / "ortho.json").read_text())
        self.mosaic = cv2.imread(str(HERE / "ortho" / "ortho.jpg"), cv2.IMREAD_COLOR)
        if self.mosaic is None:
            raise SystemExit("camera/ortho/ortho.jpg missing: run camera/fetch_ortho.py first")
        self.e_min, self.n_max, self.mpp = meta["easting_min"], meta["northing_max"], meta["m_per_px"]
        self.credit = meta["credit"] + " (modified)"
        self.tan_half = math.tan(math.radians(fov_deg) / 2)
        self.snapshot = snapshot
        self.heading_up = heading_up
        self.sign = None          # (bgra texture, lat, lon, metres per texture pixel)
        self.sign_mtime = None

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.on_global, px4_qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_local, px4_qos)
        self.image_pub = self.create_publisher(Image, "/drone/camera/image_raw", 2)
        self.info_pub = self.create_publisher(CameraInfo, "/drone/camera/camera_info", 2)
        self.global_pos = None
        self.local_pos = None
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"aerial camera: {fov_deg:.0f} deg FOV, {rate:.0f} Hz, "
                               f"{'heading' if heading_up else 'north'} up, "
                               f"mosaic {meta['width']}x{meta['height']} at {self.mpp} m/px")

    def on_global(self, msg: VehicleGlobalPosition) -> None:
        self.global_pos = msg

    def on_local(self, msg: VehicleLocalPosition) -> None:
        self.local_pos = msg

    def to_px(self, lat: float, lon: float):
        e, n = to_utm33(lat, lon)
        return np.array([(e - self.e_min) / self.mpp, (self.n_max - n) / self.mpp])

    def render(self, lat: float, lon: float, height: float, heading: float) -> np.ndarray:
        p = self.to_px(lat, lon)
        # Mosaic pixels per metre towards true north and east (UTM grid north is ~1.7 deg off).
        m_lat = 1.0 / 111320.0
        m_lon = m_lat / math.cos(math.radians(lat))
        north = self.to_px(lat + m_lat, lon) - p
        east = self.to_px(lat, lon + m_lon) - p
        fwd = math.cos(heading) * north + math.sin(heading) * east
        right = -math.sin(heading) * north + math.cos(heading) * east
        s = 2 * height * self.tan_half / WIDTH     # metres of ground per output pixel
        cx, cy = WIDTH / 2, HEIGHT / 2
        # Output pixel (u, v) -> mosaic pixel: p + right*(u-cx)*s + fwd*(cy-v)*s
        m = np.array([[right[0] * s, -fwd[0] * s, p[0] - right[0] * s * cx + fwd[0] * s * cy],
                      [right[1] * s, -fwd[1] * s, p[1] - right[1] * s * cx + fwd[1] * s * cy]])
        return cv2.warpAffine(self.mosaic, m, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=NO_IMAGERY)

    def load_sign(self) -> None:
        """Pick up sign.py's control file when it appears, changes or goes away."""
        try:
            mtime = os.stat(SIGN_CONTROL).st_mtime
        except FileNotFoundError:
            self.sign, self.sign_mtime = None, None
            return
        if mtime == self.sign_mtime:
            return
        self.sign_mtime = mtime
        try:
            ctl = json.loads(pathlib.Path(SIGN_CONTROL).read_text())
            tex = cv2.imread(ctl["image"], cv2.IMREAD_UNCHANGED)
            if tex is None or tex.shape[2] != 4:
                raise ValueError(f"{ctl['image']}: not an RGBA image")
            self.sign = (tex, float(ctl["lat"]), float(ctl["lon"]), float(ctl["width_m"]) / tex.shape[1])
            self.get_logger().info(f"sign placed at {ctl['lat']:.5f}, {ctl['lon']:.5f}")
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as e:
            self.get_logger().warning(f"ignoring {SIGN_CONTROL}: {e}")
            self.sign = None

    def draw_sign(self, img: np.ndarray, lat: float, lon: float, height: float, view: float) -> None:
        """Blend the ground marking into the frame, in the same geometry as render()."""
        if self.sign is None:
            return
        tex, s_lat, s_lon, res = self.sign
        r = 6371000.0
        dn = math.radians(lat - s_lat) * r                                        # drone - sign, metres
        de = math.radians(lon - s_lon) * r * math.cos(math.radians(s_lat))
        a = (2 * height * self.tan_half / WIDTH) / res                            # texture px per frame px
        c, s = math.cos(view), math.sin(view)
        cx, cy = WIDTH / 2, HEIGHT / 2
        th, tw = tex.shape[:2]
        # Frame pixel (u, v) -> texture pixel (texture north up, centred on the sign).
        m = np.array([[a * c, -a * s, tw / 2 + de / res - a * c * cx + a * s * cy],
                      [a * s, a * c, th / 2 - dn / res - a * c * cy - a * s * cx]])
        warped = cv2.warpAffine(tex, m, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
        alpha = warped[:, :, 3:4].astype(np.float32) / 255.0
        img[:] = (img * (1 - alpha) + warped[:, :, :3] * alpha).astype(np.uint8)

    def overlay(self, img: np.ndarray, height: float, heading: float) -> None:
        hud = f"ALT {height:5.1f} m   HDG {math.degrees(heading) % 360:03.0f}"
        cv2.rectangle(img, (0, 0), (250, 26), (0, 0, 0), -1)
        cv2.putText(img, hud, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, HEIGHT - 20), (WIDTH, HEIGHT), (0, 0, 0), -1)
        cv2.putText(img, self.credit, (8, HEIGHT - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)

    def tick(self) -> None:
        g, loc = self.global_pos, self.local_pos
        if g is None or loc is None:
            return
        height = max(-loc.z, MIN_HEIGHT_M)
        heading = loc.heading if math.isfinite(loc.heading) else 0.0
        view = heading if self.heading_up else 0.0          # direction at the top of the frame
        img = self.render(g.lat, g.lon, height, view)
        self.load_sign()
        self.draw_sign(img, g.lat, g.lon, height, view)
        self.overlay(img, height, heading)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "drone_camera"
        msg.height, msg.width = HEIGHT, WIDTH
        msg.encoding = "bgr8"
        msg.step = WIDTH * 3
        msg.data = img.tobytes()
        self.image_pub.publish(msg)
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width = HEIGHT, WIDTH
        f = (WIDTH / 2) / self.tan_half
        info.k = [f, 0.0, WIDTH / 2, 0.0, f, HEIGHT / 2, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, WIDTH / 2, 0.0, 0.0, f, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)
        if self.snapshot:
            cv2.imwrite(self.snapshot, img)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fov", type=float, default=90.0, help="horizontal field of view, degrees")
    ap.add_argument("--rate", type=float, default=5.0, help="frames per second")
    ap.add_argument("--snapshot", help="also write every frame to this JPEG file")
    ap.add_argument("--heading-up", action="store_true", help="top of the frame = drone heading (default: north)")
    args = ap.parse_args()
    rclpy.init()
    node = AerialCamera(args.fov, args.rate, args.snapshot, args.heading_up)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
