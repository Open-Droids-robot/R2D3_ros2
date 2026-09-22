#!/usr/bin/env bash
# The one way the rogent REPL is started inside the container.
#
#   rogent-run.sh [--wait] [main.py args...]
#
# Both entry points call this and nothing else, so the sim-ready guard and the
# environment rogent needs exist in exactly one place:
#   - `./droid rogent` on the host execs it (no --wait): a stalled or absent
#     clock is a hard failure with remediation text, as before.
#   - the agent pane on the noVNC desktop (gui-start.sh) runs it with --wait:
#     the pane comes up with the display, minutes before the sim does, so it
#     polls until the clock advances and then starts the REPL.
#
# Sim-ready guard: /clock must be ADVANCING, not merely present. The known
# failure mode in this repo is a stalled clock with every process still alive
# (headless render death), so two samples are compared rather than asserting
# a single message arrived. The session inherits the container-wide
# RMW_IMPLEMENTATION=rmw_zenoh_cpp, so this also proves the zenoh graph rogent
# is about to join actually carries sim traffic.
set -eu

wait_for_clock="no"
if [ "${1:-}" = "--wait" ]; then
  wait_for_clock="yes"
  shift
fi

# The ROS setup scripts read variables they do not set; relax -u around
# sourcing only (same rationale as launch-sim.sh). The workspace overlay is
# sourced separately, and late: on a fresh container it does not exist until
# launch-sim.sh's colcon build has run, which in --wait mode is after we start.
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.sh
set -u
source_workspace() {
  set +u
  # shellcheck disable=SC1091
  [ -f /ws/install/setup.sh ] && . /ws/install/setup.sh
  set -u
}

# Sampling commands get </dev/null: they must never read the stdin a piped goal
# (scripted PoC runs through `./droid rogent`) is arriving on -- main.py must.
clock_advancing() {
  a="$(timeout 5 ros2 topic echo /clock --once 2>/dev/null </dev/null || true)"
  sleep 1
  b="$(timeout 5 ros2 topic echo /clock --once 2>/dev/null </dev/null || true)"
  [ -n "$a" ] && [ "$a" != "$b" ]
}

if [ "$wait_for_clock" = "yes" ]; then
  echo "rogent: waiting for the simulation clock to advance (the sim is still coming up) ..."
  until clock_advancing; do
    sleep 2
  done
else
  clock_advancing || {
    echo "rogent: the simulation clock is not advancing. Launch the sim first" >&2
    echo "        (./droid up --rogent --mujoco) and wait for it to come up, then retry." >&2
    exit 6
  }
fi

# Sourced, not bare python3: rogent's sim nav agent is an rclpy node, and rclpy
# only exists on sys.path inside the ROS environment (the derived image
# deliberately filters PyPI's fake rclpy out of the pip install). Without this
# the nav2 bridge import fails and navigation silently degrades to stubs.
source_workspace
cd /rogent
exec python3 main.py "$@"
