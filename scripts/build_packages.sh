#!/usr/bin/env bash
# scripts/build_packages.sh
#
# Build our ROS 2 packages (rm_ros_interfaces + the three r2d3_* packages)
# inside the ros_humble conda env. Wraps the conda activation + the CMake
# Python-finder workaround required for RoboStack + ament_cmake on this box.
#
# Usage:
#   scripts/build_packages.sh                   # build everything
#   scripts/build_packages.sh r2d3_model       # build a specific package
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_PREFIX=/usr1/home/semathew/miniforge3/envs/ros_humble
MAMBA=/usr1/home/semathew/miniforge3/bin/mamba

# The CMake Python finder in ROS 2 Humble + RoboStack + CMake≥3.31 needs
# explicit hints to discover the conda env's Python. Without these,
# `find_package(Python ... COMPONENTS Interpreter Development NumPy)` fails
# even though all the pieces (Python.h, libpython3.11.so, numpy headers)
# are present in the env.
CMAKE_ARGS=(
  -DPython_ROOT_DIR="$ROS_PREFIX"
  -DPython_EXECUTABLE="$ROS_PREFIX/bin/python3"
  -DPython_FIND_VIRTUALENV=ONLY
  -DPython_FIND_STRATEGY=LOCATION
)

PACKAGES=("$@")
if [[ ${#PACKAGES[@]} -eq 0 ]]; then
  PACKAGES=(rm_ros_interfaces r2d3_task_interfaces r2d3_model_interfaces r2d3_model)
fi

cd "$REPO_ROOT"
echo "[build_packages] repo:     $REPO_ROOT"
echo "[build_packages] env:      $ROS_PREFIX"
echo "[build_packages] packages: ${PACKAGES[*]}"

"$MAMBA" run -n ros_humble bash -c "
  cd '$REPO_ROOT'
  colcon build \\
    --packages-select ${PACKAGES[*]} \\
    --merge-install \\
    --cmake-args ${CMAKE_ARGS[*]}
"
echo
echo "[build_packages] done. Source the workspace before running:"
echo "  source install/setup.bash"
