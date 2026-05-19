# `Docker/isaac/`

Isaac-Sim-specific Docker assets for the `isaac-sim-v1` branch. **Sits alongside the existing `Docker/docker/`** (ROS2-only setup, distros Foxy/Humble/Jazzy) — does not modify or replace it.

## Planned contents

| File | Purpose |
|---|---|
| `Dockerfile.isaac-sim` | Extends `nvcr.io/nvidia/isaac-sim:5.1.0` with ROS2 bridge config + `r2d3_sim` wrappers |
| `compose.isaac.yaml` | Standalone compose file. Networks the Isaac Sim service with the existing `r2d3-humble` service from `Docker/docker-compose.yml`. Pins to GPU 1 by default. |
| `run.isaac.sh` | Quick `docker run` wrapper (X11/GPU/volume mounts) for one-off launches |
| `nvidia.env.example` | Template for NGC credentials (do NOT commit your real key) |

## Why a separate subdir
- Keeps the upstream Gazebo workflow untouched on the `Docker/docker/` side
- Easy `git diff` between `main` and `isaac-sim-v1`
- Hackathon users who only want ROS2 don't pull the Isaac Sim image stack

Builds are **deferred** until `/usr0` cleanup is confirmed by the lab admin (Docker data-root lives there and is currently ~97 % full).
