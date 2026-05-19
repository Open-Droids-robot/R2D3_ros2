# R2D3 Isaac Sim Branch (`isaac-sim-v1`)

This branch adds an **NVIDIA Isaac Sim** layer on top of the existing R2D3 ROS2 packages — preserving the upstream Gazebo workflow on `main` while building a new simulation + evaluation stack for hackathon use.

## V1 Scope (10 weeks)

- USD asset of R2D3 converted from `ros2_rm_robot/dual_rm_description/` URDFs
- Python wrappers (`isaac_sim/r2d3_sim/`) for scene / robot / sensor APIs
- ROS2 Humble bridge (joint control, sensors, TF) via Isaac Sim's built-in bridge
- MoveIt2 integration reusing `ros2_rm_robot/dual_rm_moveit_config/`
- 4-task evaluation suite: Pick & Place, Stacking, Bimanual Handoff, Vision-Guided Grasp
- `r2d3-eval` CLI for deterministic scoring
- Docker image (`Docker/isaac/`)
- MkDocs site under `docs/`
- Leaderboard (stretch)

**Deferred to V2:** Woosh AGV chassis (stationary base for V1) and dexterous hands (parallel grippers only in V1).

## Layout of new content on this branch

| Path | Purpose |
|---|---|
| `isaac_sim/usd/` | R2D3 USD asset (output of URDF→USD conversion) |
| `isaac_sim/scenes/` | Scene scripts for the 4 eval tasks |
| `isaac_sim/r2d3_sim/` | Python package: scene/robot/sensor APIs |
| `isaac_sim/tests/` | pytest suite for the Python wrappers |
| `tasks/` | Task definitions (YAML/Python) for the eval suite |
| `r2d3_eval/` | `r2d3-eval` CLI source |
| `scripts/isaacsim_ros2.sh` | Launcher: runs a script with the `isaac` env's Python and the `ros_humble` env's message libs |
| `scripts/build_packages.sh` | Wrapper around `colcon build` with the CMake Python-finder hints pinned |
| `scripts/urdf_to_usd.py` | Wrapper around `urdf_usd_converter` (Isaac Sim 6.0+) for URDF→USD conversion |
| `docs/` | MkDocs source (setup, architecture, roadmap) |
| `scripts/` | Local helpers: env check, VNC start, Isaac launch wrappers |
| `.vscode/` | Workspace settings + recommended extensions |

The upstream `ros2_*` packages and `Docker/docker/` are **unchanged** on this branch.

## Where to start
- Setup: [`docs/setup.md`](docs/setup.md)
- Roadmap: [`docs/roadmap.md`](docs/roadmap.md)
- Architecture: [`docs/architecture.md`](docs/architecture.md)

## Source references
- Upstream Gazebo workflow: see this repo's [`README.md`](README.md) and [`Docker/QUICKSTART.md`](Docker/QUICKSTART.md)
- Hackathon scaffold reference: <https://github.com/intrinsic-dev/aic>
