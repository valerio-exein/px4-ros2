#!/usr/bin/env python3
"""Place or remove the attacker's ground marking in the simulated camera's world.

    python3 camera/sign.py place [--north 0] [--east 0]   # metres from home (the Colosseum)
    python3 camera/sign.py remove

Writes or deletes /tmp/drone-sign.json, which camera/aerial_camera.py checks on every frame:
the marking (camera/sign.png, made by make_sign.py when missing) appears on the ground at
that spot, 50 x 28 m, text facing north. Default spot: the Colosseum's arena, under the
drone's takeoff point, so it is in view right after takeoff.
"""
import argparse
import json
import math
import os
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
CONTROL = "/tmp/drone-sign.json"
HOME = (41.8902, 12.4922)            # run-px4.sh home (Colosseum)
WIDTH_M, HEIGHT_M = 50.0, 28.0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=["place", "remove"])
    ap.add_argument("--north", type=float, default=0.0, help="metres north of home")
    ap.add_argument("--east", type=float, default=0.0, help="metres east of home")
    args = ap.parse_args()
    if args.action == "remove":
        if os.path.exists(CONTROL):
            os.remove(CONTROL)
        print("marking removed")
        return
    image = HERE / "sign.png"
    if not image.exists():
        subprocess.run([sys.executable, str(HERE / "make_sign.py"), str(image)], check=True)
    r = 6371000.0
    lat = HOME[0] + math.degrees(args.north / r)
    lon = HOME[1] + math.degrees(args.east / (r * math.cos(math.radians(HOME[0]))))
    tmp = CONTROL + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"image": str(image), "lat": lat, "lon": lon, "width_m": WIDTH_M, "height_m": HEIGHT_M}, f)
    os.replace(tmp, CONTROL)
    print(f"marking placed at N {args.north:+.0f} m, E {args.east:+.0f} m from home ({lat:.5f}, {lon:.5f})")


if __name__ == "__main__":
    main()
