#!/usr/bin/env bash
# scripts/isaacsim_ros2.sh
#
# Launch a Python script with Isaac Sim 6.0 (+ optional ROS 2 Humble support).
#
# Usage:
#   scripts/isaacsim_ros2.sh my_script.py [args...]
#
# Envs (auto-detected; override via R2D3_CONDA_BASE / R2D3_ISAAC_ENV / R2D3_ROS_ENV
# or R2D3_ISAAC_PY / R2D3_ROS_PREFIX — see scripts/_conda_env.sh):
#   - isaac       (Python 3.12)  Isaac Sim 6.0 pip install + isaacsim-ros2 bridge   (REQUIRED)
#   - ros_humble  (Python 3.11)  RoboStack ros-humble-* — message libs              (OPTIONAL)
#
# The in-process SDK path needs ONLY the isaac env. The ros_humble env is needed
# solely for the ROS bridge (bring_up.py + r2d3_humble_bridge); if it's absent we
# warn and run anyway. We do NOT source ros_humble's setup.bash — that would
# prepend its Python 3.11 site-packages and corrupt Isaac's bundled rclpy; we only
# put its prefix on AMENT_PREFIX_PATH so the C++ message libs are findable.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/_conda_env.sh
source "$REPO_ROOT/scripts/_conda_env.sh"
r2d3_resolve_envs

ISAAC_PY="$R2D3_ISAAC_PY"
ROS_HUMBLE_PREFIX="$R2D3_ROS_PREFIX"
LOCAL_INSTALL="$REPO_ROOT/install"

if [[ ! -x "$ISAAC_PY" ]]; then
  echo "error: Isaac env Python not found at: $ISAAC_PY" >&2
  echo "       Detected conda base: ${R2D3_CONDA_BASE:-<none>}" >&2
  echo "       Run scripts/bootstrap.sh, or create it yourself:" >&2
  echo "         conda create -n isaac python=3.12 -y" >&2
  echo "         conda run -n isaac pip install 'isaacsim[all,extscache]==6.0.0.0' --extra-index-url https://pypi.nvidia.com" >&2
  echo "       (override the location with R2D3_ISAAC_PY=/path/to/python)" >&2
  exit 2
fi
if [[ $# -lt 1 ]]; then
  echo "usage: $0 <script.py> [args...]" >&2
  exit 1
fi

# EULA acceptance (skip the interactive prompt on first run)
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
export ACCEPT_EULA=Y

# ROS 2 plumbing — only wire ros_humble if it actually exists (SDK use doesn't need it).
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [[ -d "$ROS_HUMBLE_PREFIX" ]]; then
  export AMENT_PREFIX_PATH="$LOCAL_INSTALL:$ROS_HUMBLE_PREFIX"
  export LD_LIBRARY_PATH="$LOCAL_INSTALL/lib:$ROS_HUMBLE_PREFIX/lib:${LD_LIBRARY_PATH:-}"
else
  echo "note: ros_humble env not found at $ROS_HUMBLE_PREFIX — running without the ROS bridge." >&2
  echo "      (the in-process R2D3 SDK does not need it; only bring_up.py / the bridge do.)" >&2
  export AMENT_PREFIX_PATH="$LOCAL_INSTALL"
  export LD_LIBRARY_PATH="$LOCAL_INSTALL/lib:${LD_LIBRARY_PATH:-}"
fi

exec "$ISAAC_PY" "$@"
