#!/usr/bin/env bash
# scripts/isaacsim_ros2.sh
#
# Launch a Python script with Isaac Sim 6.0 + ROS 2 Humble support.
# Modeled on dameria's recipe at /usr1/home/dameria/isaacsim_ros2.sh.
#
# Usage:
#   scripts/isaacsim_ros2.sh my_script.py [args...]
#
# Two-env strategy (do NOT collapse into a single env):
#   - isaac       (Python 3.12)  Isaac Sim 6.0 pip install + isaacsim-ros2 bridge
#   - ros_humble  (Python 3.11)  RoboStack ros-humble-* — provides message libs
#
# Isaac's bundled rclpy expects Python 3.12 paths. Sourcing ros_humble's
# `setup.bash` would prepend 3.11 site-packages to PYTHONPATH and corrupt the
# import. We only need ros_humble's prefix on AMENT_PREFIX_PATH so the C++
# message libs (libsensor_msgs.so, libgeometry_msgs.so, etc.) are findable.

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MINIFORGE=/usr1/home/semathew/miniforge3
ISAAC_PY="$MINIFORGE/envs/isaac/bin/python"
ROS_HUMBLE_PREFIX="$MINIFORGE/envs/ros_humble"
LOCAL_INSTALL="$REPO_ROOT/install"

if [[ ! -x "$ISAAC_PY" ]]; then
  echo "error: Isaac env not found at $ISAAC_PY" >&2
  echo "       Run: mamba create -n isaac python=3.12 && pip install 'isaacsim[all]==6.0.0.0' --extra-index-url https://pypi.nvidia.com" >&2
  exit 2
fi
if [[ ! -d "$ROS_HUMBLE_PREFIX" ]]; then
  echo "error: ros_humble env not found at $ROS_HUMBLE_PREFIX" >&2
  exit 3
fi
if [[ $# -lt 1 ]]; then
  echo "usage: $0 <script.py> [args...]" >&2
  exit 1
fi

# EULA acceptance (skip the interactive prompt on first run)
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
export ACCEPT_EULA=Y

# ROS 2 plumbing
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Local workspace first, then RoboStack env. AMENT_PREFIX_PATH is the only
# thing we need from the ROS env — see header note about why we do NOT source
# its setup.bash.
export AMENT_PREFIX_PATH="$LOCAL_INSTALL:$ROS_HUMBLE_PREFIX"

# ROS C++ libraries need to be findable at runtime.
export LD_LIBRARY_PATH="$LOCAL_INSTALL/lib:$ROS_HUMBLE_PREFIX/lib:${LD_LIBRARY_PATH:-}"

# Tell Isaac where Python message modules live (built into install/lib/python3.11/site-packages
# by colcon, but the path is for cpython 3.11 not 3.12 — incompatible. We rely on
# isaacsim-ros2's own rclpy. If a user truly needs to import our messages from inside
# Isaac, we need to rebuild the interfaces packages against Python 3.12 separately.
# For now, document this limitation:
#   - r2d3_model node:                 runs inside ros_humble env (Python 3.11), OK
#   - Scripts using just Isaac Python: cannot `import r2d3_model_interfaces.msg`
# If we need that cross-Python use case, build with -DPython_EXECUTABLE pointing at
# isaac env's python and a separate install prefix.

exec "$ISAAC_PY" "$@"
