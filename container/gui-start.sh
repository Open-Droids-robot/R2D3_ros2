#!/usr/bin/env bash
# Starts the in-container GUI stack, then execs the requested command.
#
# A virtual X server plus a window manager plus noVNC is the single universal GUI
# path: identical on macOS, Jetson, cloud and Linux, with no X11 socket bind-mount,
# no DISPLAY plumbing and no VNC client to install. It also gives Gazebo a GLX
# context rather than requiring headless EGL, which has been unreliable here --
# the symptom of EGL failure is /clock silently stalling rather than a clean error.
set -eu

export DISPLAY=":1"
GEOMETRY="${DROID_GEOMETRY:-1920x1080x24}"

Xvfb "$DISPLAY" -screen 0 "$GEOMETRY" +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
for _ in $(seq 1 50); do
  xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 && break
  sleep 0.2
done
xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || {
  echo "droid: virtual X server failed to start; see /tmp/xvfb.log" >&2
  exit 1
}

# Window placement for the four-pane desktop (see container/fluxbox-apps for
# the layout). The apps file is installed fresh on every start so the image's
# copy is authoritative. The toolbar is hidden so it does not eat a strip of
# the bottom row. configVersion is seeded with it: without it fluxbox runs
# its config-update pass on first start, which rewrites init from the stock
# defaults and drops the toolbar line. Both are added once -- fluxbox rewrites
# init on exit, so a blind append would duplicate them. Fluxbox reads both
# files at startup, hence before it is launched.
# The stock style's `background:` line makes fluxbox call fbsetbg
# on every start; hsetroot is in the image so that call succeeds silently
# rather than popping fbsetbg's "no wallpaper setter" xmessage over the
# desktop (the documented `background: none` overlay did NOT prevent it on a
# cold start here, only on a restart).
mkdir -p "$HOME/.fluxbox"
cp /opt/droid/fluxbox-apps "$HOME/.fluxbox/apps"
fluxbox_init_default() { # fluxbox_init_default <key> <value>
  grep -q "^$1:" "$HOME/.fluxbox/init" 2>/dev/null ||
    echo "$1: $2" >> "$HOME/.fluxbox/init"
}
fluxbox_init_default session.configVersion 13
fluxbox_init_default session.screen0.toolbar.visible false
fluxbox >/tmp/fluxbox.log 2>&1 &
# -localhost is not optional. `-nopw` makes this an UNAUTHENTICATED VNC endpoint
# with full keyboard and mouse control, and without -localhost x11vnc binds
# 0.0.0.0:5900 -- reachable from any sibling container on the compose bridge
# network, and LAN-reachable the moment someone publishes port 5900. Bound to the
# loopback interface it is reachable only by websockify, which is the only thing
# that should ever dial it: it already connects to localhost:5900.
x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -quiet -rfbport 5900 \
  >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 \
  >/tmp/websockify.log 2>&1 &

# Rogent mode: exactly one zenoh router, owned by this session supervisor.
# gui-start runs once per container start, so single-instance is true by
# construction -- no pgrep dance. Supervised here rather than started by any
# sim launch so the router (like the display and VNC above) outlives every
# individual sim or agent process; rogent and the sim nodes all dial
# tcp/localhost:7447. launch-sim.sh gates on that port actually listening.
# DROID_ROGENT is set by container/docker-compose.rogent.yml only.
if [ "${DROID_ROGENT:-}" = "1" ]; then
  # The ROS setup scripts read variables they do not set; relax -u around
  # sourcing only (same rationale as launch-sim.sh).
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/jazzy/setup.sh
  set -u
  # Not on PATH: the setup script doesn't expose package-private executables;
  # this is the same binary `ros2 run rmw_zenoh_cpp rmw_zenohd` resolves to.
  /opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd >/tmp/rmw_zenohd.log 2>&1 &
fi

# The terminal panes of the desktop. Each is an xterm pinned to its quadrant
# (pixel offsets; the column/row counts just fill it at this font size) and
# respawned if closed, so a stray Ctrl-D or a finished REPL does not leave the
# browser view without a way back in. Both run as this (droid) user with the
# ROS environment sourced -- the workspace overlay lazily, since on a fresh
# container it only exists after the first launch's colcon build.
#
#   shell pane  (lower right) -- both modes: `ros2 topic list` and friends.
#   agent pane  (lower left)  -- rogent mode only: the same REPL `./droid
#                                rogent` gives on the host, through the same
#                                script, which waits for the sim clock to
#                                advance before starting it.
pane() { # pane <name> <geometry> <command...>
  name="$1"; geometry="$2"; shift 2
  while :; do
    xterm -name "$name" -title "$name" -geometry "$geometry" \
      -fa 'DejaVu Sans Mono' -fs 11 -bg black -fg grey90 +sb -e "$@" || true
    sleep 1
  done
}
pane shell 118x28+960+640 bash -c '
  set +u; . /opt/ros/jazzy/setup.sh
  [ -f /ws/install/setup.sh ] && . /ws/install/setup.sh
  set -u; exec bash -i' >/tmp/pane-shell.log 2>&1 &
if [ "${DROID_ROGENT:-}" = "1" ]; then
  pane rogent 118x28+0+640 /opt/droid/rogent-run.sh --wait >/tmp/pane-rogent.log 2>&1 &
fi

exec "$@"
