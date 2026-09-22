#!/usr/bin/env bash
# devcontainer.json `postCreateCommand`: runs INSIDE the dev container, once per
# container creation, as the remote user (droid), from the workspace folder.
#
# 1. Make every integrated terminal a ROS shell: source /opt/ros/jazzy and the
#    workspace install space from ~/.bashrc. ~/.bashrc is container-local (the
#    home directory is not a volume), so this is re-applied on every rebuild
#    and never leaks onto the host.
#
# 2. Build the simulation subset, exactly as every `./droid up` launch path
#    does before launching: plain `colcon build`, no --symlink-install, scoped
#    by the image's /etc/colcon/defaults.yaml. install/ is plain copies, so an
#    edit under src/ does not exist until a rebuild copies it -- the same trap
#    as on the host (see docs/container.md). This first build populates the shared
#    install volume on a fresh clone; later rebuilds are on you:
#    `colcon build --packages-select <pkg>` from /ws.
set -eu

marker="# >>> r2d3 devcontainer >>>"
if ! grep -qF "$marker" "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<'EOF'

# >>> r2d3 devcontainer >>>
# ROS 2 Jazzy plus the workspace install space in every terminal. The setup
# scripts read variables they do not set, so guard them against `set -u`.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u; . /opt/ros/jazzy/setup.bash; set -u 2>/dev/null || true
fi
if [ -f /ws/install/setup.bash ]; then
  set +u; . /ws/install/setup.bash; set -u 2>/dev/null || true
fi
[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] &&
  . /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# <<< r2d3 devcontainer <<<
EOF
  echo "post-create.sh: ROS environment added to ~/.bashrc"
fi

set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
set -u

cd /ws
echo "post-create.sh: building the simulation subset (colcon build, no --symlink-install)"
colcon build
echo "post-create.sh: done. Open a new terminal: ros2 and the workspace are sourced."
