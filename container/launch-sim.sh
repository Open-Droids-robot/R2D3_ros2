#!/usr/bin/env bash
# Rebuild the simulation subset, then launch. Every launch path rebuilds first.
#
# The workspace is built WITHOUT --symlink-install, here exactly as on the host,
# so that every existing document stays true and there is one mental model. That
# makes install/ plain copies, which means an edit under src/ does not exist until
# colcon build copies it -- and the symptom of forgetting is "this setting does
# nothing", which sends people debugging code that was never wrong. Rebuilding on
# every launch makes that impossible to trigger. It is cheap: the whole subset is
# data packages, with C++ only in r2d3_test_nodes (four files) and servo_interfaces
# (two message definitions).
set -eu

BACKEND="${1:-gz}"

# The ROS setup scripts read variables they do not always set (AMENT_TRACE_SETUP_FILES
# among them), so `set -u` turns sourcing them into an immediate exit 1 -- before this
# script reaches any of its own logic, including its unknown-backend guard. Relax `-u`
# across the sourcing only, then restore it for the code this file is responsible for.
source_ros() {
  set +u
  # shellcheck disable=SC1090
  . "$1"
  set -u
}

source_ros /opt/ros/jazzy/setup.sh
cd /ws
colcon build
source_ros /ws/install/setup.sh

launch_with_rviz() {
  "$@" &
  sim_pid=$!
  rviz_config="$(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/rviz/view.rviz"
  rviz2 -d "$rviz_config" &
  rviz_pid=$!
  # Tear the viewer down with the simulator rather than leaving it orphaned on a
  # dead graph. Stale simulator-side processes surviving a naive cleanup have
  # produced false verdicts in this repo before, so the teardown is by PID.
  trap 'kill "$sim_pid" "$rviz_pid" 2>/dev/null || true' EXIT INT TERM
  wait "$sim_pid"
}

case "$BACKEND" in
  gz)
    launch_with_rviz ros2 launch dual_rm_simulation gz_sim.launch.py
    ;;
  mujoco)
    # First launch on an UNMODIFIED tree hits the cache baked into the image and
    # starts promptly. A feature branch or local description edit changes the
    # generated robot description, so the content-addressed cache misses and the
    # full multi-minute reconversion runs. That is correct behaviour, not a hang.
    launch_with_rviz ros2 launch r2d3_mujoco mujoco_sim.launch.py
    ;;
  *)
    echo "launch-sim: unknown backend '$BACKEND' (expected gz or mujoco)" >&2
    exit 2
    ;;
esac
