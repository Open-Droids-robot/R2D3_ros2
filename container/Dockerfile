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
# The whole step is `container/prewarm-mjcf.py` (already in the image via the COPY
# above) and NOT a few lines of shell, for one decisive reason: bash command
# substitution strips trailing newlines, and `launch.substitutions.Command` does
# not. `description="$(xacro ...)"` therefore produces a description one byte
# shorter than the one the launch produces, which is a different sha256, which is a
# different cache key -- a pre-warm that always misses, with every build-time
# assertion still passing because a cache does exist. The driver uses the real
# `Command` substitution and reads the launch file's own DeclareLaunchArgument
# defaults, then ASSERTS that the checksum it computes equals the checksum file the
# converter wrote. That assertion catches converter-side divergence, a
# CONVERTER_ARGS_VERSION bump, and a stale/foreign checksum -- but NOT the driver's
# hand-composed xacro argument list drifting from the launch file's (both sides of
# the assertion derive from the same hand-composed description, so an argument
# ADDED to mujoco_sim.launch.py's Command([...]) slips through it). That drift is
# caught by container/test/test_container_config.py's
# TestPrewarmXacroArgsMatchLaunchFile instead; see the script's docstring.
USER droid
RUN source /opt/ros/jazzy/setup.bash; \
    source /ws/install/setup.bash; \
    export HOME=/home/droid; \
    python3 /ws/src/R2D3_ros2/container/prewarm-mjcf.py
USER root

COPY container/entrypoint.sh container/gui-start.sh container/launch-sim.sh /opt/droid/
RUN chmod +x /opt/droid/*.sh

# The virtual X server that gui-start.sh creates. It MUST live in the image env,
# not only in gui-start.sh's own `export`: the launch runs via `docker compose
# exec`, and an exec session inherits the image's ENV but NOT the runtime exports
# the entrypoint/gui-start process made for PID 1. Without this, Gazebo's Qt GUI
# finds no display, the xcb platform plugin fails to initialise, and the whole
# `gz sim` process aborts on startup -- taking the server, /clock and the
# controller_manager with it. Placing it here also gives `./droid shell` users a
# working display for rviz2/rqt. Kept in lockstep with gui-start.sh's `:1`.
ENV DISPLAY=:1

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
