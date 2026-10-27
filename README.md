# PX4 + ROS 2 (Lyrical) demo setup

PX4 autopilot in simulation (Docker, SIH physics, no Gazebo) talking to ROS 2 Lyrical through
the Micro XRCE-DDS Agent. Follows https://docs.px4.io/main/en/ros2/user_guide, adapted to
ROS 2 Lyrical / Ubuntu 26.04 (the guide targets Jazzy).

```
PX4 SITL (docker, host network)          this machine
  uxrce_dds_client  ── UDP 8888 ──►  MicroXRCEAgent  ── DDS (domain 0) ──►  ROS 2 nodes
                                                                           /fmu/out/*  from PX4
                                                                           /fmu/in/*   to PX4
```

## Layout

| Path | What |
| --- | --- |
| `setup.sh` | prepares a fresh machine: workspace sources at pinned versions + build, QGroundControl, imagery, PX4 image |
| `demo.sh` | starts / stops the whole demo in tmux (see `README-demo.md`) |
| `px4-env.sh` | source in every terminal: ROS 2 Lyrical + this workspace, `ROS_DOMAIN_ID=0` |
| `flight/` | LLM pilot, fixed-route flight, position feed for the geofence, guided scenario |
| `camera/` | simulated aerial camera, imagery download, attacker's ground marking |
| `security/ros2tap-drone.toml` | ros2tap policy: the Vittoriano no-fly zone |
| `patches/` | `px4_ros_com` patch for Lyrical (applied by `setup.sh`) |
| `ws/src/px4_msgs` | PX4 message definitions, pinned to `57f88be` (= PX4 `37e0cb3f`, same `msg/` as the container's firmware) |
| `ws/src/Micro-XRCE-DDS-Agent` | agent **v3.0.2** (the guide says v2.4.3, which needs Fast DDS 2; Lyrical ships Fast DDS 3) |
| `ws/src/px4_ros_com` | examples (`main`), **patched**: `ament_target_dependencies()` (removed in Lyrical) replaced by `target_link_libraries()` |

The PX4 container is `px4io/px4-sitl:latest`: PX4 v1.18.0 dev (`main`, `61907962`), model
`sihsim_quadx`. If the PX4 image changes, re-pin `px4_msgs` to the commit that matches its `msg/`
(or run PX4's message translation node).

## Run

Use bash for ROS setup files (or `source px4-env.sh` from bash/zsh).

```bash
# 1. PX4, home at the Colosseum in Rome (tmux session "px4", window "px4sitl")
./run-px4.sh        # = docker run --rm -it --network host -e PX4_HOME_LAT/LON/ALT ... px4io/px4-sitl

# 2. Agent (tmux session "px4", window "agent")
cd /home/vaioco/workspace/exein.io/physicalai/demo/px4-ros2
source px4-env.sh && MicroXRCEAgent udp4 -p 8888
#    -> "session established ... 127.0.0.1"; the PX4 console shows
#       "successfully created rt/fmu/out/... data writer"

# 3. Check from ROS 2
source px4-env.sh
ros2 topic list | grep /fmu/            # 69 topics: /fmu/out/* (31) and /fmu/in/* (38)
ros2 launch px4_ros_com sensor_combined_listener.launch.py   # live IMU data

# 4. UI: QGroundControl v5.1.5 (AppImage, sha256-checked; tmux session "px4", window "qgc")
#    Listens on UDP 14550, where PX4 already sends MAVLink: connects by itself.
qgc/QGroundControl-x86_64.AppImage

# 5. Flight: take off, loop around the Colosseum, return, land (~3.5 min; window "flight")
python3 flight/takeoff_and_fly.py            # --alt 60 --radius 150 --points 12 (defaults)

#    (PX4's own example only takes off to 5 m and hovers: ros2 run px4_ros_com offboard_control)
```

`flight/takeoff_and_fly.py` streams offboard position setpoints on `/fmu/in/trajectory_setpoint`
and commands on `/fmu/in/vehicle_command`, and reads `/fmu/out/vehicle_local_position_v1` and
`/fmu/out/vehicle_status_v4` (this PX4 build's topic names; px4_ros_com's Python example uses the
old unversioned names). Waypoints are fixed.

## Camera and live view

A simulated downward-looking camera over real aerial imagery of Rome (SIH has no 3D world):

```bash
python3 -I camera/fetch_ortho.py     # once: 4 x 4 km around the Colosseum, 0.5 m/px -> camera/ortho/
python3 camera/aerial_camera.py      # /drone/camera/image_raw + camera_info, 640x480, 5 Hz, 90 deg FOV
ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw
```

Imagery: Regione Lazio geoportal WMS, "Ortofoto AGEA v. 2023", CC BY 4.0. The credit
"Ortofoto AGEA 2023 - Regione Lazio, CC BY 4.0 (modified)" is drawn on every frame. Fetched in the
layer's native EPSG:25833 (UTM 33N): the server swaps the colour bands when asked for EPSG:4326.
`camera/geo.py` converts lat/lon to UTM (no pyproj needed).

## LLM pilot

```bash
python3 flight/llm_pilot.py                  # interactive: pilot> fly to the Pantheon and circle it
python3 flight/llm_pilot.py "take off, fly to the Roman Forum, circle it once, come back and land"
python3 flight/llm_pilot.py --dry-run "..."  # plan only;  --no-camera: don't send the frame
```

Command + drone state + named places (Colosseum, Arch of Constantine, Palatine, Roman Forum, Circus
Maximus, Vittoriano, Trevi, Pantheon) + the current camera frame go to gemma4 (OLLAMA_ENDPOINT,
default 192.168.2.114). It answers JSON: what it sees, its reasoning, and actions from a closed set
(takeoff, goto, orbit, hover, return_home, land). The app's own safety check rejects plans outside
10-120 m altitude, 2 km from home, orbit radius 20-300 m, 3 laps, 60 s hover.

- The offboard stream runs from arming until the land command; left on during a landing it keeps
  PX4's land detector from declaring touchdown.
- If the pilot stops while airborne, PX4 loses offboard after 1 s; with no RC its fallback ends in
  Return-to-Launch (fly home, land). A new pilot takes over in offboard on its next command.
- The simulated battery drains to 50% and stays there (SIM_BAT_MIN_PCT).

Subscribers to PX4 topics need a sensor-data QoS (best effort); `ros2 topic echo` may need
`--qos-reliability best_effort`.

## Setup on a new machine

Needs Ubuntu 26.04 with ROS 2 Lyrical, Docker and tmux; this folder next to the ros2tap clone
with the drone geofence:

```
demo/
  px4-ros2/          this repo
  ros2tap-drone/     ros2tap, branch drone-geofence
```

```bash
./setup.sh                                   # sources, build (~10 min), QGroundControl, imagery, PX4 image
(cd ../ros2tap-drone && cargo build --release)   # needs clang and libbpf-dev
./demo.sh
```

ros2tap writes into process memory: the kernel must have lockdown `none`
(`cat /sys/kernel/security/lockdown`). `setup.sh` is safe to re-run; `REBUILD=1 ./setup.sh`
rebuilds the workspace.

## Notes

- Domain: PX4 publishes on ROS domain 0 (`UXRCE_DDS_DOM_ID`). The VisionNav demo uses 42, so the
  two don't see each other. Another ROS machine on the LAN using domain 0 would see `/fmu/*`.
- Only one agent per UDP port.
