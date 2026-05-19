#!/usr/bin/env bash
# scripts/run_isaac.sh — launch the Isaac Sim container.
# Currently a PLACEHOLDER: the Docker image hasn't been built yet (waiting on /usr0 cleanup).
#
# When the image is ready, this will wrap `docker compose ... up isaac-sim` with the
# correct -f files, DISPLAY, and GPU pin. For now it performs a dry-run that prints
# the equivalent command and confirms prerequisites.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILES=(
  "-f" "$REPO_ROOT/Docker/isaac/compose.isaac.yaml"
)

# Optional: stack with the upstream Gazebo/ROS2 compose. Off by default for V1
# because the Isaac container already includes the ROS2 bridge.
if [[ "${WITH_ROS2_HUMBLE:-0}" == "1" ]]; then
  COMPOSE_FILES=(
    "-f" "$REPO_ROOT/Docker/docker-compose.yml"
    "${COMPOSE_FILES[@]}"
  )
fi

# Prefer GPU 1 by default (GPU 0 commonly busy on Riddle).
export NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-1}"
# Default DISPLAY to :1 if user has a TurboVNC session running and didn't export one.
export DISPLAY="${DISPLAY:-:1}"

echo "[run_isaac] repo:           $REPO_ROOT"
echo "[run_isaac] DISPLAY:        $DISPLAY"
echo "[run_isaac] GPU pin:        device=$NVIDIA_VISIBLE_DEVICES"
echo "[run_isaac] compose files:  ${COMPOSE_FILES[*]}"

# Sanity checks before launch
"$REPO_ROOT/scripts/check_env.sh" | tail -20

# Block actual launch until image exists. Replace this guard with `docker compose up`
# once `r2d3-isaac-sim:dev` has been built.
if ! docker image inspect r2d3-isaac-sim:dev >/dev/null 2>&1; then
  cat <<EOF >&2

[run_isaac] r2d3-isaac-sim:dev not built yet — refusing to launch.

To build (only after /usr0 has free space):
  docker compose ${COMPOSE_FILES[*]} build isaac-sim

To launch (interactive shell inside the container):
  docker compose ${COMPOSE_FILES[*]} run --rm isaac-sim

To launch with WebRTC streaming (no X required):
  docker compose ${COMPOSE_FILES[*]} run --rm isaac-sim /isaac-sim/runheadless.native.sh -v

EOF
  exit 2
fi

exec docker compose "${COMPOSE_FILES[@]}" run --rm isaac-sim "$@"
