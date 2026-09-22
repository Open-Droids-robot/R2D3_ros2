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

# Rogent mode: "router up before sim nodes" is checked, not hoped. Every node
# in this container speaks rmw_zenoh through the tcp/localhost:7447 router that
# gui-start.sh supervises; launching without it means nodes that start, log
# nothing useful, and match nothing. Bash's /dev/tcp probe needs no client
# installed. Bounded: gui-start raced ahead of us only briefly if at all.
if [ "${DROID_ROGENT:-}" = "1" ]; then
  router_up="no"
  for _ in $(seq 1 50); do
    if (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
      router_up="yes"
      break
    fi
    sleep 0.2
  done
  if [ "$router_up" != "yes" ]; then
    echo "launch-sim: rogent mode, but no zenoh router is listening on :7447 after 10s." >&2
    echo "            See /tmp/rmw_zenohd.log; the router is started by gui-start.sh." >&2
    exit 3
  fi
fi

cd /ws
colcon build
source_ros /ws/install/setup.sh

launch_with_rviz() {
  "$@" &
  sim_pid=$!
  rviz_config="$(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/rviz/view.rviz"
  # use_sim_time is not optional here. Both backends publish /clock and stamp TF in
  # simulation time, which starts near zero; rviz2 defaults to false and would read
  # every transform as hours out of date, so the display comes up empty with "no
  # transform" errors -- which reads as "the container is broken" rather than as a
  # misconfigured clock.
  rviz2 -d "$rviz_config" --ros-args -p use_sim_time:=true &
  rviz_pid=$!
  # Tear the viewer down with the simulator rather than leaving it orphaned on a
  # dead graph. Be precise about what this delivers: SIGTERM to the two launcher
  # PIDs and nothing else. `ros2 launch` and `rviz2` shut down their own managed
  # processes on that signal, but gz's detached helpers (gz sim server/gui and the
  # ruby wrapper) have survived naive cleanup in this repo before and may outlive
  # it. The real backstop is the container boundary -- stopping the container reaps
  # everything in its PID namespace -- so a wedged simulator is a `droid down`, not
  # something this trap can promise to fix.
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
    #
    # Rogent mode needs the FULL stack: the agent's nav2 bridge dispatches to
    # Nav2's /navigate_to_pose, so the sim-only launch leaves it dead on
    # arrival. bringup_sim.launch.py is the sim counterpart of the robot's
    # `ros2 launch rogent rogent.launch.py` -- sim + Nav2 behind the readiness
    # gate. use_rviz:=false because launch_with_rviz owns the viewer here;
    # use_moveit:=false because sim manipulation is faked (rogent-v3#2), so
    # move_group would only burn CPU beside the VLM. Default mode is unchanged.
    if [ "${DROID_ROGENT:-}" = "1" ]; then
      launch_with_rviz ros2 launch r2d3_mujoco bringup_sim.launch.py \
        use_rviz:=false use_moveit:=false
    else
      launch_with_rviz ros2 launch r2d3_mujoco mujoco_sim.launch.py
    fi
    ;;
  *)
    echo "launch-sim: unknown backend '$BACKEND' (expected gz or mujoco)" >&2
    exit 2
    ;;
esac
