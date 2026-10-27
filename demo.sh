#!/usr/bin/env bash
# Start the PX4 + ROS 2 LLM-pilot demo in one tmux session ("px4").
#
#   ./demo.sh [start]   start everything and attach (default)
#   ./demo.sh attach    re-attach to a running demo
#   ./demo.sh stop      stop everything (the PX4 container is removed: the sim starts fresh next time)
#
# Windows: shell, px4sitl (PX4 console, home at the Colosseum), agent (Micro XRCE-DDS),
# camera (simulated aerial camera), view (rqt_image_view), qgc (QGroundControl), posefeed
# (drone position for the geofence), and "demo", where the demo happens:
#   left: the LLM pilot (type commands at pilot>)   top right: ros2tap (asks the sudo password)
#   bottom right: the guided sign-injection scenario (press Enter for each step)
#
# Settings (environment variables):
#   OLLAMA_ENDPOINT   default http://192.168.2.114:11434     OLLAMA_MODEL   default gemma4:12b
#   DISPLAY           default :0 (for QGroundControl and the camera viewer)
#   GEOFENCE=0        no ros2tap and no position feed (the drone is not protected)
set -euo pipefail

SESSION=px4
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV="source '$ROOT/px4-env.sh'"
OLLAMA_ENDPOINT="${OLLAMA_ENDPOINT:-http://192.168.2.114:11434}"
OLLAMA_MODEL="${OLLAMA_MODEL:-gemma4:12b}"
GUI_DISPLAY="${DISPLAY:-:0}"
QGC="$ROOT/qgc/QGroundControl-x86_64.AppImage"
GEOFENCE="${GEOFENCE:-1}"
# ros2tap with the drone geofence: the separate clone next to this folder (branch drone-geofence).
TAP_REL="../ros2tap-drone/target/release/ros2tap"
TAP_PATTERN='^\.\./ros2tap-drone/target/release/ros2tap'

die() { echo "demo.sh: $*" >&2; exit 1; }

running() { tmux has-session -t "$SESSION" 2>/dev/null; }

attach() {
  if [ ! -t 1 ]; then
    echo "no terminal; attach with: tmux attach -t $SESSION"
  elif [ -n "${TMUX:-}" ]; then tmux switch-client -t "$SESSION"; else tmux attach -t "$SESSION"; fi
}

# Run a command in a new window's bash; the shell stays open after it exits, so errors stay visible.
window() {
  local name="$1" cmd="$2"
  tmux new-window -d -t "$SESSION:" -n "$name" -c "$ROOT" bash
  tmux send-keys -t "$SESSION:$name" "$ENV && $cmd" C-m
}

start() {
  command -v tmux >/dev/null || die "tmux is not installed"
  command -v docker >/dev/null || die "docker is not installed"
  if running; then
    echo "demo already running; attaching (./demo.sh stop to restart)"
    attach
    return
  fi
  docker ps --format '{{.Names}}' | grep -qx px4-sitl \
    && die "a px4-sitl container is already running outside tmux; stop it first (docker stop px4-sitl)"
  docker image inspect px4io/px4-sitl:latest >/dev/null 2>&1 || die "image px4io/px4-sitl:latest missing (docker pull)"
  [ -f "$ROOT/ws/install/setup.bash" ] || die "ROS workspace not built (see README.md, Build)"
  [ -f "$ROOT/camera/ortho/ortho.jpg" ] || die "camera imagery missing (python3 -I camera/fetch_ortho.py)"
  if [ "$GEOFENCE" = 1 ]; then
    [ -x "$ROOT/$TAP_REL" ] || die "$TAP_REL not built (cd ../ros2tap-drone && cargo build --release)"
    pgrep -f "$TAP_PATTERN" >/dev/null && die "ros2tap-drone already running; stop it first (./demo.sh stop)"
  fi
  python3 "$ROOT/camera/sign.py" remove >/dev/null   # no marking left over from a previous run

  # Nodes that need PX4 wait until its topics are visible in ROS 2.
  local wait_px4="echo 'waiting for PX4...' \
&& until timeout 5 ros2 topic list 2>/dev/null | grep -q /fmu/out/vehicle_status_v4; do sleep 2; done"

  tmux new-session -d -s "$SESSION" -n shell -c "$ROOT" bash
  tmux send-keys -t "$SESSION:shell" "$ENV && echo 'PX4 LLM-pilot demo. Stop: ./demo.sh stop'" C-m
  window px4sitl "./run-px4.sh"
  window agent "MicroXRCEAgent udp4 -p 8888"
  window camera "$wait_px4 && python3 camera/aerial_camera.py"
  window view "DISPLAY=$GUI_DISPLAY ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw"
  if [ -x "$QGC" ]; then
    window qgc "DISPLAY=$GUI_DISPLAY '$QGC'"
  else
    echo "QGroundControl not found at $QGC: no map view (see README.md)"
  fi
  [ "$GEOFENCE" = 1 ] && window posefeed "$wait_px4 && python3 flight/pose_feed.py"

  # The demo window: pilot | ros2tap over the guided scenario.
  tmux new-window -d -t "$SESSION:" -n demo -c "$ROOT" bash
  local pilot_pane tap_pane scen_pane
  pilot_pane=$(tmux display-message -p -t "$SESSION:demo" '#{pane_id}')
  tap_pane=$(tmux split-window -d -h -l 45% -t "$pilot_pane" -c "$ROOT" -P -F '#{pane_id}' bash)
  scen_pane=$(tmux split-window -d -v -l 40% -t "$tap_pane" -c "$ROOT" -P -F '#{pane_id}' bash)
  tmux send-keys -t "$pilot_pane" \
    "$ENV && $wait_px4 && OLLAMA_ENDPOINT='$OLLAMA_ENDPOINT' OLLAMA_MODEL='$OLLAMA_MODEL' python3 flight/llm_pilot.py" C-m
  if [ "$GEOFENCE" = 1 ]; then
    tmux send-keys -t "$tap_pane" \
      "$ENV && sudo $TAP_REL --config security/ros2tap-drone.toml > /tmp/ros2tap-drone-events.jsonl" C-m
  else
    tmux send-keys -t "$tap_pane" "echo 'GEOFENCE=0: ros2tap not started, the drone is NOT protected'" C-m
  fi
  tmux send-keys -t "$scen_pane" "$ENV && flight/scenario.sh '$pilot_pane'" C-m

  tmux select-window -t "$SESSION:demo"
  if [ "$GEOFENCE" = 1 ] && ! sudo -n true 2>/dev/null; then
    tmux select-pane -t "$tap_pane"
    echo "started: type the sudo password in the ros2tap pane (top right), then press Enter in the scenario pane (bottom right)"
  else
    tmux select-pane -t "$scen_pane"
    echo "started: press Enter in the scenario pane (bottom right) once the pilot shows pilot> (~15 s)"
  fi
  attach
}

stop() {
  # ros2tap runs as root: Ctrl-C in its pane may not reach it, and the pane cleanup below
  # cannot kill it. SIGINT is its clean exit (probe detached, status written).
  if pgrep -f "$TAP_PATTERN" >/dev/null; then
    sudo -n pkill -INT -f "$TAP_PATTERN" 2>/dev/null || sudo pkill -INT -f "$TAP_PATTERN" || true
  fi
  python3 "$ROOT/camera/sign.py" remove >/dev/null 2>&1 || true
  running || { echo "demo not running"; return; }
  for w in $(tmux list-windows -t "$SESSION" -F '#W'); do
    tmux send-keys -t "$SESSION:$w" C-c
  done
  # The container is --rm: stopping it removes it (and resets the simulated drone).
  docker stop -t 10 px4-sitl >/dev/null 2>&1 || true
  sleep 2
  local pid
  for pid in $(tmux list-panes -s -t "$SESSION" -F '#{pane_pid}'); do
    pkill -9 -s "$pid" 2>/dev/null || true
  done
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "demo stopped"
}

case "${1:-start}" in
  start) start ;;
  attach) running && attach || die "demo not running" ;;
  stop) stop ;;
  *) die "usage: ./demo.sh [start|attach|stop]" ;;
esac
