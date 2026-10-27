#!/usr/bin/env bash
# Start PX4 SITL (SIH physics, quadcopter) in Docker with home at the Colosseum, Rome.
#
#   ./run-px4.sh                 foreground, PX4 shell (pxh>) attached; Ctrl-C or `shutdown` stops it
#
# Host network: PX4 reaches the XRCE-DDS agent on 127.0.0.1:8888 and QGroundControl on UDP 14550.
# The container is --rm: parameters changed at runtime are lost when it stops.
set -euo pipefail

PX4_HOME_LAT="${PX4_HOME_LAT:-41.8902}"   # Colosseum
PX4_HOME_LON="${PX4_HOME_LON:-12.4922}"
PX4_HOME_ALT="${PX4_HOME_ALT:-25}"        # ground elevation, m above sea level

exec docker run --rm -it --network host --name px4-sitl \
  -e PX4_HOME_LAT="$PX4_HOME_LAT" -e PX4_HOME_LON="$PX4_HOME_LON" -e PX4_HOME_ALT="$PX4_HOME_ALT" \
  px4io/px4-sitl:latest
