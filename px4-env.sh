# Shared environment for the PX4 + ROS 2 demo. Source it (bash or zsh), don't run it:
#   source px4-env.sh

if [ -n "${ZSH_VERSION:-}" ]; then
  _px4_env_self="${(%):-%x}"
  _px4_env_sh=zsh
else
  _px4_env_self="${BASH_SOURCE[0]}"
  _px4_env_sh=bash
fi
PX4_DEMO_ROOT="$(cd "$(dirname "$_px4_env_self")" && pwd)"
export PX4_DEMO_ROOT

# PX4's uXRCE-DDS client publishes on ROS domain UXRCE_DDS_DOM_ID (PX4 default 0); the
# ROS 2 side must use the same domain to see the /fmu topics.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [ -z "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/lyrical/setup.$_px4_env_sh" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/lyrical/setup.$_px4_env_sh"
fi
[ -n "${ROS_DISTRO:-}" ] || echo "px4-env: no ROS 2 found under /opt/ros" >&2
if [ -f "$PX4_DEMO_ROOT/ws/install/setup.$_px4_env_sh" ]; then
  # shellcheck disable=SC1090
  source "$PX4_DEMO_ROOT/ws/install/setup.$_px4_env_sh"
fi

unset _px4_env_self _px4_env_sh
