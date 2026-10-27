#!/usr/bin/env bash
# Prepare this demo on a fresh machine (ROS 2 Lyrical and Docker already installed).
#
#   ./setup.sh
#
# 1. ROS 2 workspace: clone px4_msgs, px4_ros_com and the Micro XRCE-DDS Agent at the pinned
#    versions, patch px4_ros_com for Lyrical, build (px4_msgs takes ~10 min).
# 2. QGroundControl AppImage (checksum verified) into qgc/.
# 3. Aerial imagery for the camera into camera/ortho/ (Regione Lazio WMS, CC BY 4.0).
# 4. The PX4 SITL image (docker pull).
# Every step is skipped when its result is already there. No sudo.
#
# The geofence also needs ros2tap with the drone geofence (branch drone-geofence) built in
# ../ros2tap-drone (cargo build --release); see README.md.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# px4_msgs must match the PX4 firmware's msg/ (px4io/px4-sitl, PX4 61907962): px4_msgs 57f88be
# was generated from PX4 37e0cb3f, and no msg/ file changed in between.
PX4_MSGS_REF=57f88be3c01a8c593c23507e4ca136767115ca22
PX4_ROS_COM_REF=86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af
AGENT_REF=v3.0.2          # v3: built on Fast DDS 3, which Lyrical ships (the PX4 guide's v2.4.3 needs 2.x)
QGC_URL=https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-x86_64.AppImage
QGC_SHA256=9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344
PX4_IMAGE=px4io/px4-sitl:latest

log() { printf '[setup] %s\n' "$*"; }

clone() {  # clone <dir> <url> <ref>
  local dir="ws/src/$1"
  if [ ! -d "$dir/.git" ]; then
    log "cloning $1 at $3"
    git clone -q "$2" "$dir"
  fi
  git -C "$dir" fetch -q --tags origin
  git -C "$dir" checkout -q "$3"
}

mkdir -p ws/src
clone px4_msgs https://github.com/PX4/px4_msgs.git "$PX4_MSGS_REF"
clone px4_ros_com https://github.com/PX4/px4_ros_com.git "$PX4_ROS_COM_REF"
clone Micro-XRCE-DDS-Agent https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_REF"

# Lyrical removed ament_target_dependencies(); the patch switches px4_ros_com to target_link_libraries().
if git -C ws/src/px4_ros_com apply --check "$ROOT/patches/px4_ros_com-lyrical.patch" 2>/dev/null; then
  log "patching px4_ros_com for Lyrical"
  git -C ws/src/px4_ros_com apply "$ROOT/patches/px4_ros_com-lyrical.patch"
fi

if [ ! -f ws/install/setup.bash ] || [ "${REBUILD:-0}" = 1 ]; then
  log "building the ROS 2 workspace (px4_msgs takes ~10 min)"
  (cd ws && bash -c 'source /opt/ros/lyrical/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release')
fi

if [ ! -x qgc/QGroundControl-x86_64.AppImage ]; then
  log "downloading QGroundControl v5.1.5"
  mkdir -p qgc
  curl -sSL -o qgc/QGroundControl-x86_64.AppImage.part "$QGC_URL"
  echo "$QGC_SHA256  qgc/QGroundControl-x86_64.AppImage.part" | sha256sum -c --quiet
  mv qgc/QGroundControl-x86_64.AppImage.part qgc/QGroundControl-x86_64.AppImage
  chmod +x qgc/QGroundControl-x86_64.AppImage
fi

if [ ! -f camera/ortho/ortho.jpg ]; then
  log "downloading the aerial imagery (Regione Lazio, CC BY 4.0)"
  bash -c 'source /opt/ros/lyrical/setup.bash && python3 -I camera/fetch_ortho.py'
fi

docker image inspect "$PX4_IMAGE" >/dev/null 2>&1 || { log "pulling $PX4_IMAGE"; docker pull -q "$PX4_IMAGE"; }

[ -x ../ros2tap-drone/target/release/ros2tap ] \
  || log "note: ../ros2tap-drone/target/release/ros2tap missing: the geofence will not start (see README.md)"
log "done: ./demo.sh"
