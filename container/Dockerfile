# One image, ROS 2 Jazzy, for linux/amd64 and linux/arm64.
#
# FROM ros:jazzy-ros-base rather than osrf/ros:jazzy-desktop-full: the osrf images
# are published for amd64 only and therefore cannot run natively on Apple Silicon
# or a Jetson. Jazzy is the only distro here -- Gazebo Harmonic pairs with Jazzy,
# mujoco_ros2_control has no Humble binary, and Foxy is end-of-life.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# GUI stack, software GL, and the handful of tools the entry point and the
# developer need. Image size is explicitly not a constraint here; functionality
# takes priority.
RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb x11vnc novnc websockify fluxbox x11-utils x11-xserver-utils \
      libgl1-mesa-dri mesa-utils libglx-mesa0 \
      python3-colcon-common-extensions python3-vcstool \
      sudo git curl wget nano less bash-completion util-linux \
    && rm -rf /var/lib/apt/lists/*

# The Debian noVNC package ships vnc.html but no index.html, so the bare
# http://localhost:6080/ a developer will actually type serves a 404. Task 4's
# `up` prints that URL; make it work.
RUN ln -sf vnc.html /usr/share/novnc/index.html

# The runtime user. Starts at 1000 and is remapped to the host's ids by the
# entrypoint, so a developer whose uid is not 1000 needs no local image rebuild.
# noble's base image ships an 'ubuntu' user at 1000 that must go first.
RUN if getent passwd ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi && \
    groupadd --gid 1000 droid && \
    useradd --uid 1000 --gid 1000 -m -s /bin/bash droid && \
    usermod -aG video,audio droid && \
    echo 'droid ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/droid && \
    chmod 0440 /etc/sudoers.d/droid

# Narrow every colcon invocation to the simulation subset. See the file's own
# comment for why this is a defaults file and not COLCON_IGNORE markers.
COPY container/colcon-defaults.yaml /etc/colcon/defaults.yaml
ENV COLCON_DEFAULTS_FILE=/etc/colcon/defaults.yaml

WORKDIR /ws
COPY --chown=droid:droid . /ws/src/R2D3_ros2

# Dependencies come from rosdep over the package manifests, so the manifests stay
# the single source of truth rather than a hand-maintained apt list.
#
# Scoped to the SIMULATION packages, via colcon's own resolution of the defaults
# file above, rather than `--from-paths src`. A bare --from-paths would walk all
# 29 discoverable packages and pull the ZED and RealSense SDK dependencies back
# in -- exactly the multi-gigabyte payload the ignore set exists to keep out, and
# it would do it silently while `colcon list` still reported a clean 15.
#
# warehouse_ros_mongo is the one key with no jazzy/noble binary; it is a MoveIt
# warehouse backend the simulation never loads. Nothing else is skipped: a second
# unresolvable key means a real dependency changed and must be investigated, not
# added to this list.
RUN apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths $(colcon list --paths-only) --ignore-src -y \
      --rosdistro jazzy --skip-keys "warehouse_ros_mongo" \
    && rm -rf /var/lib/apt/lists/*

# A built workspace is required, not incidental: pre-warming the MuJoCo cache
# means running the description through xacro and the converter at image build
# time, which needs the workspace installed first.
#
# Built WITHOUT --symlink-install, matching the host exactly, so install/ is
# plain copies and the one documented mental model stays true in both places.
RUN source /opt/ros/jazzy/setup.bash && colcon build && chown -R droid:droid /ws

# Pre-warm the MuJoCo converter venv and the converted scene into the ROS user
# state directory, which is exposed as a volume seeded from the image.
#
# The benefit is deliberately scoped: the conversion is content-addressed over the
# generated robot description plus the world file, so this is a cache HIT only for
# an unmodified checkout at this revision. A developer on a feature branch pays the
# full reconversion, correctly.
#
# Three things this step does NOT do, each for a reason:
#
#  * It does not run `mujoco_sim.launch.py`. The launch brings up
#    mujoco_ros2_control, which opens a MuJoCo window unless headless -- and
#    `headless` is an argument to the xacro, so passing headless:=true to dodge
#    that changes the generated description and therefore the content hash. The
#    cache would be warmed under a key the default launch never asks for: a
#    pre-warm that is always a miss, and silently so. Driving ensure_mjcf.py
#    directly with the launch file's own defaults (arm_model 65b, headless false,
#    nav_empty.xml) warms the key the default launch will actually look up.
#
#  * It does not `sleep` for a fixed interval. A guessed duration is either wasted
#    build time or a truncated conversion; polling for the cache to appear takes
#    exactly as long as the conversion takes, and the timeout is a backstop rather
#    than the mechanism.
#
#  * It does not use `kill %1`, and it does not go through `ros2 run`. Job control
#    is off in a non-interactive shell, so `%1` never resolves; and `ros2 run`
#    Popen()s the script as a child, so $! would name the launcher rather than the
#    converter driver and SIGTERM would stop the wrong process. Invoking the
#    installed script directly makes $! the process that must actually be signalled.
#    ensure_mjcf.py restores default signal handling before it latches the model
#    and spins, so SIGTERM ends it cleanly.
#
# Note the statement separators below: the steps are joined with `;` and not `&&`,
# because `a && b &` backgrounds the whole list -- $! would be a subshell and the
# kill would never reach the converter.
#
# The trailing assertions are load-bearing. Without them a converter that failed
# after printing an error would still produce a "successful" image whose first
# launch silently reconverts.
USER droid
RUN source /opt/ros/jazzy/setup.bash; \
    source /ws/install/setup.bash; \
    set -e; \
    export HOME=/home/droid; \
    prefix="$(ros2 pkg prefix r2d3_mujoco)"; \
    share="$prefix/share/r2d3_mujoco"; \
    cache="$HOME/.ros/r2d3_mujoco/65b"; \
    description="$(xacro "$share/urdf/r2d3_mujoco.urdf.xacro" arm_model:=65b headless:=false)"; \
    "$prefix/lib/r2d3_mujoco/ensure_mjcf.py" \
      --robot-description "$description" \
      --world "$share/worlds/nav_empty.xml" \
      --model 65b & \
    prewarm_pid=$!; \
    deadline=$((SECONDS + 1200)); \
    while [ "$SECONDS" -lt "$deadline" ]; do \
      if [ -f "$cache/checksum" ] && [ -f "$cache/mujoco_description_formatted.xml" ]; then break; fi; \
      kill -0 "$prewarm_pid" 2>/dev/null || break; \
      sleep 5; \
    done; \
    kill "$prewarm_pid" 2>/dev/null || true; \
    wait "$prewarm_pid" 2>/dev/null || true; \
    test -f "$cache/checksum" \
      || { echo "prewarm: no checksum -- the MuJoCo conversion did not complete" >&2; exit 1; }; \
    test -f "$cache/mujoco_description_formatted.xml" \
      || { echo "prewarm: no MJCF -- the MuJoCo conversion did not complete" >&2; exit 1; }; \
    test -x "$HOME/.ros/ros2_control/.venv/bin/python3" \
      || { echo "prewarm: converter venv missing -- first launch would rebuild it" >&2; exit 1; }
USER root

COPY container/entrypoint.sh container/gui-start.sh container/launch-sim.sh /opt/droid/
RUN chmod +x /opt/droid/*.sh

# Bridge networking with published ports, never host networking -- host networking
# is Linux-only and would break macOS. Outbound HTTPS is all that is required;
# distributed ROS across machines is explicitly out of scope, so the graph stays
# inside the container.
ENV ROS_LOCALHOST_ONLY=1 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    XDG_CACHE_HOME=/home/droid/.cache \
    HF_HOME=/home/droid/.cache/huggingface \
    TORCH_HOME=/home/droid/.cache/torch

EXPOSE 6080
ENTRYPOINT ["/opt/droid/entrypoint.sh"]
CMD ["sleep", "infinity"]
