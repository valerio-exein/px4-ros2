#!/usr/bin/env python3
"""LLM pilot: natural-language commands -> LLM flight plan -> PX4 offboard flight over ROS 2.

    source px4-env.sh
    python3 flight/llm_pilot.py                       # interactive: type commands, 'quit' to exit
    python3 flight/llm_pilot.py "fly to the Pantheon and circle it, then come back and land"
    python3 flight/llm_pilot.py --dry-run "..."       # plan only, no flight

The LLM (Ollama, OLLAMA_ENDPOINT / OLLAMA_MODEL, default gemma4:12b on 192.168.2.114) gets the
command, the drone's state and a list of named places, and must answer with JSON: a short
reasoning and a list of actions from a closed set (takeoff, goto, orbit, hover, return_home,
land). When camera/aerial_camera.py is running, the current frame of /drone/camera/image_raw
goes to the LLM too, and the plan includes what it sees ("observation"). validate() then checks the plan against fixed limits before anything flies; this is the
application's own safety layer, not an enforcement point the rest of the system can rely on.

Positions are PX4's local NED frame in metres from home (x north, y east, z down).
"""
import argparse
import base64
import collections
import json
import math
import os
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
import rclpy.executors
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

ENDPOINT = os.getenv("OLLAMA_ENDPOINT", "http://192.168.2.114:11434").rstrip("/")
MODEL = os.getenv("OLLAMA_MODEL", "gemma4:12b")

# Home when PX4 does not report its reference (run-px4.sh starts PX4 at the Colosseum).
DEFAULT_HOME = (41.8902, 12.4922)

# Named places the pilot may use (approximate coordinates, good to a few tens of metres).
PLACES = {
    "colosseum": ("Colosseum", 41.8902, 12.4922),
    "arch_of_constantine": ("Arch of Constantine", 41.8898, 12.4906),
    "palatine_hill": ("Palatine Hill", 41.8893, 12.4875),
    "roman_forum": ("Roman Forum", 41.8925, 12.4853),
    "circus_maximus": ("Circus Maximus", 41.8861, 12.4851),
    "altare_della_patria": ("Altare della Patria (Vittoriano)", 41.8946, 12.4831),
    "trevi_fountain": ("Trevi Fountain", 41.9009, 12.4833),
    "pantheon": ("Pantheon", 41.8986, 12.4769),
}

# Safety limits applied to every plan before it flies.
ALT_MIN, ALT_MAX = 10.0, 120.0       # m above home (EU open category ceiling: 120 m)
MAX_RANGE = 2000.0                   # m from home, horizontal
RADIUS_MIN, RADIUS_MAX = 20.0, 300.0
LAPS_MAX = 3
HOVER_MAX = 60.0                     # s
DEFAULT_ALT = 50.0
REACHED_M = 3.0
ORBIT_POINTS = 12                    # waypoints per lap
STALL_S = 10.0                       # no progress towards a waypoint this long: plan stopped
                                     # (e.g. a runtime geofence holding the drone)

ACTIONS = ["takeoff", "goto", "orbit", "hover", "return_home", "land"]

CAMERA_TOPIC = "/drone/camera/image_raw"   # camera/aerial_camera.py (downward-looking)
CAMERA_FOV_DEG = 90.0


def local_offset(home, lat, lon):
    """(north, east) metres of lat/lon from home (equirectangular, fine at city scale)."""
    r = 6371000.0
    north = math.radians(lat - home[0]) * r
    east = math.radians(lon - home[1]) * r * math.cos(math.radians(home[0]))
    return north, east


# ── ROS 2 side: state + offboard setpoint loop ─────────────────────────────────

class Pilot(Node):
    """Streams offboard setpoints at 10 Hz and works through a queue of steps."""

    def __init__(self) -> None:
        super().__init__("llm_pilot")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_position, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self.on_status, qos)
        self.create_subscription(Image, CAMERA_TOPIC, self.on_image, 2)
        self.position = None
        self.status = None
        self.image = None                    # latest camera frame (sensor_msgs/Image)
        self.steps = collections.deque()     # ("arm",) ("wp", (x, y, z)) ("hold", s) ("land",)
        self.idle = threading.Event()
        self.idle.set()
        self.target = None                   # current setpoint (x, y, z)
        # Offboard stream (heartbeat + setpoint): on from before arming until the land command.
        # Left on during a landing, it keeps PX4's land detector from declaring touchdown.
        self.streaming = False
        self.stream_since = 0.0
        self.step_started = 0.0
        self.step_sent = False
        self.best_dist = math.inf            # closest approach to the current waypoint
        self.create_timer(0.1, self.tick)

    # state
    def on_position(self, msg: VehicleLocalPosition) -> None:
        self.position = msg

    def on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def on_image(self, msg: Image) -> None:
        self.image = msg

    def camera_jpeg_b64(self):
        """Latest camera frame as base64 JPEG, or None (no camera running)."""
        msg = self.image
        if msg is None or msg.encoding != "bgr8":
            return None
        frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(jpg.tobytes()).decode() if ok else None

    def armed(self) -> bool:
        return bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def here(self):
        p = self.position
        return (p.x, p.y, p.z) if p else (0.0, 0.0, 0.0)

    def home(self):
        p = self.position
        if p and p.xy_global and (p.ref_lat or p.ref_lon):
            return (p.ref_lat, p.ref_lon)
        return DEFAULT_HOME

    # commands
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

    def run(self, steps) -> None:
        # Queue first: cleared before the steps are in, the 10 Hz loop could see an empty
        # queue and mark the plan done at once.
        self.steps.extend(steps)
        self.idle.clear()

    def tick(self) -> None:
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        if self.target is None and self.position is not None:
            self.target = self.here()

        # Offboard heartbeat + setpoint: PX4 needs the stream before and during offboard.
        if self.streaming and self.target is not None:
            mode = OffboardControlMode()
            mode.position = True
            mode.timestamp = now_us
            self.mode_pub.publish(mode)
            sp = TrajectorySetpoint()
            sp.position = [float(v) for v in self.target]
            sp.yaw = self.heading_to(self.target)
            sp.timestamp = now_us
            self.setpoint_pub.publish(sp)

        if not self.steps:
            if not self.idle.is_set():
                self.idle.set()
            return
        step = self.steps[0]
        kind = step[0]
        if kind == "arm":
            if not self.streaming:
                self.target = self.here()
                self.streaming = True
                self.stream_since = time.monotonic()
            if self.armed() and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.next_step()
            elif time.monotonic() - self.stream_since < 1.0:
                pass  # PX4 only accepts offboard after about a second of setpoints
            elif not self.step_sent or time.monotonic() - self.step_started > 3.0:
                self.target = self.here()
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)   # 6 = offboard
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.step_sent = True
                self.step_started = time.monotonic()
        elif kind == "wp":
            self.target = step[1]
            if self.position is None:
                return
            d = math.dist(self.here(), step[1])
            if d < REACHED_M:
                self.next_step()
            elif not self.step_sent or d < self.best_dist - 1.0:
                self.step_sent = True            # progress: closest distance so far
                self.best_dist = d
                self.step_started = time.monotonic()
            elif time.monotonic() - self.step_started > STALL_S:
                x, y, z = step[1]
                print(f"  BLOCKED: no progress towards N {x:+.0f} m, E {y:+.0f} m for {STALL_S:.0f} s "
                      f"({d:.0f} m away); plan stopped, holding here", flush=True)
                self.steps.clear()
                self.target = self.here()
                self.step_sent = False
        elif kind == "hold":
            if not self.step_sent:
                self.step_sent = True
                self.step_started = time.monotonic()
            elif time.monotonic() - self.step_started >= step[1]:
                self.next_step()
        elif kind == "land":
            if not self.step_sent:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.streaming = False
                self.step_sent = True
            elif self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.target = self.here()
                self.next_step()

    def next_step(self) -> None:
        done = self.steps.popleft()
        self.step_sent = False
        if done[0] == "wp":
            x, y, z = done[1]
            print(f"  reached N {x:+.0f} m, E {y:+.0f} m, alt {-z:.0f} m", flush=True)
        elif done[0] == "land":
            print("  landed and disarmed", flush=True)

    def heading_to(self, target) -> float:
        if self.position is None or math.hypot(target[0] - self.position.x, target[1] - self.position.y) < 5.0:
            return float("nan")  # keep current heading
        return math.atan2(target[1] - self.position.y, target[0] - self.position.x)


# ── LLM side: plan, validate, expand into steps ────────────────────────────────

PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "observation": {"type": "string"},
        "reasoning": {"type": "string"},
        "actions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "type": {"type": "string", "enum": ACTIONS},
                    "place": {"type": "string", "enum": list(PLACES) + ["home"]},
                    "north": {"type": "number"},
                    "east": {"type": "number"},
                    "alt": {"type": "number"},
                    "radius": {"type": "number"},
                    "laps": {"type": "integer"},
                    "seconds": {"type": "number"},
                },
                "required": ["type"],
            },
        },
    },
    "required": ["reasoning", "actions"],
}


def plan_prompt(command: str, state: dict, places: dict, camera: str | None) -> str:
    place_lines = "\n".join(f"- {pid}: {name}, N {n:+.0f} m, E {e:+.0f} m from home ({math.hypot(n, e):.0f} m away)"
                            for pid, (name, n, e) in places.items())
    camera_part = (f"\nThe attached image is the drone's downward-looking camera right now: {camera}.\n"
                   if camera else "\nNo camera image is available.\n")
    return f"""You are the flight planner of a quadcopter drone over Rome. Turn the operator's
command into a flight plan made only of these actions:

- takeoff: climb to "alt" metres above home (needed first when the drone is on the ground)
- goto: fly to "place" (a place id below, or "home") or to "north"/"east" metres from home, at "alt"
- orbit: circle "place" (or "north"/"east") at "radius" metres, "laps" times, at "alt"
- hover: hold position for "seconds"
- return_home: fly back over home
- land: land where the drone is

Limits: alt {ALT_MIN:.0f}-{ALT_MAX:.0f} m, every point within {MAX_RANGE:.0f} m of home, orbit radius
{RADIUS_MIN:.0f}-{RADIUS_MAX:.0f} m, at most {LAPS_MAX} laps, hover at most {HOVER_MAX:.0f} s. If no altitude is
given use {DEFAULT_ALT:.0f} m. Only land if the operator asks to land or to finish.

Known places (home is the takeoff point, at the Colosseum):
{place_lines}

Drone state: {json.dumps(state)}
{camera_part}
Operator command: {command}

Answer with JSON: "observation" (one sentence: what the camera shows, or "no camera"),
"reasoning" (one or two sentences) and "actions" (in order)."""


def ask_llm(prompt: str, image_b64: str | None) -> dict:
    body = {"model": MODEL, "prompt": prompt, "format": PLAN_SCHEMA, "stream": False,
            "think": False, "keep_alive": -1, "options": {"temperature": 0.1}}
    if image_b64:
        body["images"] = [image_b64]
    req = urllib.request.Request(f"{ENDPOINT}/api/generate", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=300) as resp:
        data = json.load(resp)
    return json.loads(data["response"])


def validate(plan: dict, places: dict, airborne: bool, alt_now: float):
    """Check the plan against the limits. Returns (actions, problems); problems reject the plan.

    An action without an altitude flies at the altitude the drone has at that point of the plan.
    """
    problems, actions = [], []
    cur_alt = alt_now if airborne else DEFAULT_ALT
    for i, a in enumerate(plan.get("actions", []), 1):
        kind = a.get("type")
        if kind not in ACTIONS:
            problems.append(f"action {i}: unknown type {kind!r}")
            continue
        a = dict(a)
        if "alt" in a or kind in ("takeoff", "goto", "orbit"):
            alt = float(a.get("alt", cur_alt))
            if not ALT_MIN <= alt <= ALT_MAX:
                problems.append(f"action {i} ({kind}): altitude {alt:.0f} m outside {ALT_MIN:.0f}-{ALT_MAX:.0f} m")
            a["alt"] = cur_alt = alt
        if kind in ("goto", "orbit"):
            place = a.get("place")
            if place == "home":
                a["north"], a["east"] = 0.0, 0.0
            elif place:
                if place not in places:
                    problems.append(f"action {i} ({kind}): unknown place {place!r}")
                    continue
                a["north"], a["east"] = places[place][1], places[place][2]
            elif "north" not in a or "east" not in a:
                problems.append(f"action {i} ({kind}): needs a place or north/east")
                continue
            reach = math.hypot(a["north"], a["east"]) + (float(a.get("radius", 100.0)) if kind == "orbit" else 0.0)
            if reach > MAX_RANGE:
                problems.append(f"action {i} ({kind}): {reach:.0f} m from home, limit {MAX_RANGE:.0f} m")
        if kind == "orbit":
            r = float(a.get("radius", 100.0))
            laps = int(a.get("laps", 1))
            if not RADIUS_MIN <= r <= RADIUS_MAX or not 1 <= laps <= LAPS_MAX:
                problems.append(f"action {i} (orbit): radius {r:.0f} m / {laps} laps outside limits")
            a["radius"], a["laps"] = r, laps
        if kind == "hover":
            s = float(a.get("seconds", 10.0))
            if not 0 < s <= HOVER_MAX:
                problems.append(f"action {i} (hover): {s:.0f} s outside 0-{HOVER_MAX:.0f} s")
            a["seconds"] = s
        actions.append(a)
    # A plan that moves while the drone is on the ground needs a takeoff first.
    if actions and not airborne and actions[0]["type"] not in ("takeoff", "land"):
        alt = next((a["alt"] for a in actions if "alt" in a), DEFAULT_ALT)
        actions.insert(0, {"type": "takeoff", "alt": alt, "added": "takeoff added: drone on the ground"})
    return actions, problems


def to_steps(actions, start):
    """Expand validated actions into executor steps, starting from position `start` (x, y, z)."""
    steps, (x, y, z) = [], start
    for a in actions:
        kind = a["type"]
        if kind == "takeoff":
            steps += [("arm",), ("wp", (x, y, -a["alt"]))]
            z = -a["alt"]
        elif kind in ("goto", "return_home"):
            if kind == "return_home":
                a = {**a, "north": 0.0, "east": 0.0}
            z = -a.get("alt", -z)
            x, y = a["north"], a["east"]
            steps.append(("wp", (x, y, z)))
        elif kind == "orbit":
            z = -a["alt"]
            cx, cy, r = a["north"], a["east"], a["radius"]
            # Join the circle at the point nearest to where we are, then go round.
            start_angle = math.atan2(y - cy, x - cx)
            n = ORBIT_POINTS * a["laps"]
            for k in range(n + 1):
                ang = start_angle + 2 * math.pi * k / ORBIT_POINTS
                steps.append(("wp", (cx + r * math.cos(ang), cy + r * math.sin(ang), z)))
            x, y = steps[-1][1][0], steps[-1][1][1]
        elif kind == "hover":
            steps.append(("hold", a["seconds"]))
        elif kind == "land":
            steps.append(("land",))
            z = 0.0
    return steps


def describe(a: dict) -> str:
    where = a.get("place") or (f"N {a['north']:+.0f} m, E {a['east']:+.0f} m" if "north" in a else "")
    parts = [a["type"], where,
             f"alt {a['alt']:.0f} m" if "alt" in a else "",
             f"radius {a['radius']:.0f} m x{a['laps']}" if a["type"] == "orbit" else "",
             f"{a['seconds']:.0f} s" if a["type"] == "hover" else "",
             f"({a['added']})" if "added" in a else ""]
    return " ".join(p for p in parts if p)


def handle(pilot: Pilot, command: str, dry_run: bool, use_camera: bool = True) -> None:
    home = pilot.home()
    places = {pid: (name, *local_offset(home, lat, lon)) for pid, (name, lat, lon) in PLACES.items()}
    x, y, z = pilot.here()
    airborne = pilot.armed() and -z > 2.0
    state = {"airborne": airborne, "north_m": round(x), "east_m": round(y), "alt_m": round(-z)}
    image = pilot.camera_jpeg_b64() if use_camera else None
    camera = None
    if image:
        heading = pilot.position.heading if pilot.position and math.isfinite(pilot.position.heading) else 0.0
        across = 2 * max(-z, 1.5) * math.tan(math.radians(CAMERA_FOV_DEG) / 2)
        camera = (f"north is at the top of the image (the drone faces {math.degrees(heading) % 360:.0f} deg "
                  f"from north), the image is about {across:.0f} m across, centred under the drone")
    print(f"\n> {command}\n  asking {MODEL}{' with camera image' if image else ''} ...", flush=True)
    t0 = time.monotonic()
    try:
        plan = ask_llm(plan_prompt(command, state, places, camera), image)
    except Exception as e:  # network, HTTP or JSON errors: report and wait for the next command
        print(f"  planner error: {e}", flush=True)
        return
    if image:
        print(f"  sees: {plan.get('observation', '').strip()}", flush=True)
    print(f"  plan ({time.monotonic() - t0:.0f} s): {plan.get('reasoning', '').strip()}", flush=True)
    actions, problems = validate(plan, places, airborne, -z)
    for i, a in enumerate(actions, 1):
        print(f"   {i}. {describe(a)}", flush=True)
    if problems:
        print("  REJECTED by safety check:", *problems, sep="\n   - ", flush=True)
        return
    if dry_run or not actions:
        return
    steps = to_steps(actions, (x, y, z))
    # Airborne but not (or no longer) flying offboard from this node, e.g. a new pilot process
    # or after a PX4 failsafe: take over in offboard first ("arm" also engages offboard).
    in_offboard = pilot.streaming and pilot.status and pilot.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
    if airborne and steps and steps[0][0] not in ("arm", "land") and not in_offboard:
        steps.insert(0, ("arm",))
    pilot.run(steps)
    pilot.idle.wait()
    print("  plan done", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("commands", nargs="*", help="commands to run in order (none: interactive)")
    ap.add_argument("--dry-run", action="store_true", help="plan and check only, do not fly")
    ap.add_argument("--no-camera", action="store_true", help=f"do not send the {CAMERA_TOPIC} frame to the LLM")
    args = ap.parse_args()

    rclpy.init()
    pilot = Pilot()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(pilot)

    def spin() -> None:
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass  # rclpy.try_shutdown() below: the normal way out

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    for _ in range(50):  # wait up to 5 s for PX4 state
        if pilot.position is not None and pilot.status is not None:
            break
        time.sleep(0.1)
    if pilot.position is None:
        print("no PX4 state on /fmu/out (PX4 + agent running? source px4-env.sh?)", file=sys.stderr)
        if not args.dry_run:
            sys.exit(1)
    print(f"LLM pilot ready: {MODEL} at {ENDPOINT}, home {pilot.home()[0]:.4f}, {pilot.home()[1]:.4f}", flush=True)
    try:
        if args.commands:
            for c in args.commands:
                handle(pilot, c, args.dry_run, not args.no_camera)
        else:
            while True:
                c = input("\npilot> ").strip()
                if c in ("quit", "exit"):
                    break
                if c:
                    handle(pilot, c, args.dry_run, not args.no_camera)
    except (KeyboardInterrupt, EOFError):
        if pilot.armed():
            print("\ninterrupted: landing", flush=True)
            pilot.steps.clear()
            pilot.run([("land",)])
            pilot.idle.wait(timeout=180)
    finally:
        # Shut the context down first: it wakes the spinning thread, which then exits on its
        # own. Stopping the executor from here while it spins can crash rclpy.
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        pilot.destroy_node()


if __name__ == "__main__":
    main()
