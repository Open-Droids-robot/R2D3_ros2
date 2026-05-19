# Setup

> Reproduce the development environment on Riddle (or any Ubuntu 22.04 / 24.04 host with NVIDIA GPU + driver ≥ 535). Verified end-to-end on 2026-05-19 against Isaac Sim 6.0.0.0 + ROS 2 Humble (RoboStack).

The deployment uses **three colocated conda envs** under one Miniforge install — not Docker containers. See [`architecture.md`](architecture.md) for the rationale.

## Prerequisites

- NVIDIA driver ≥ 535 (`nvidia-smi` works)
- ~25 GB free disk under your home directory
- A working internet connection to `pypi.nvidia.com` and `conda-forge`

No `sudo` required after Miniforge is in place.

## Clone

```bash
git clone https://github.com/Open-Droids-robot/R2D3_ros2.git r2d3_isaac
cd r2d3_isaac
git checkout isaac-sim-v1
```

## 1. Miniforge

```bash
# Skip if you already have miniforge in your $HOME.
wget -qO miniforge.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash miniforge.sh -b -p "$HOME/miniforge3"
"$HOME/miniforge3/bin/conda" config --set auto_activate_base false
```

## 2. The three envs

### `isaac` — Isaac Sim 6.0 (Python 3.12)

```bash
~/miniforge3/bin/mamba create -n isaac -y python=3.12 pip cmake git pkg-config
~/miniforge3/envs/isaac/bin/python -m pip install --no-input \
    "isaacsim[all]==6.0.0.0" \
    --extra-index-url https://pypi.nvidia.com
```

~10 GB. Includes `isaacsim-ros2` (bundled bridge), `urdf-usd-converter` (CLI), and PyTorch with CUDA 12.8.

Verify:

```bash
OMNI_KIT_ACCEPT_EULA=YES \
  ~/miniforge3/envs/isaac/bin/python -c "import isaacsim; print('OK')"
```

### `ros_humble` — RoboStack ROS 2 Humble (Python 3.11)

```bash
~/miniforge3/bin/mamba create -n ros_humble -y -c conda-forge python=3.11
~/miniforge3/bin/mamba install -n ros_humble -y \
    -c conda-forge -c robostack-staging \
    ros-humble-desktop ros-humble-rmw-fastrtps-cpp \
    colcon-common-extensions rosdep \
    "cmake<4"     # IMPORTANT: CMake 4.x breaks ament_cmake's FindPython on this stack
```

~4 GB.

### `r2d3` — host-side eval tooling (Python 3.10)

Already created during initial setup (see top-level `README.md`). If not:

```bash
~/miniforge3/bin/mamba create -n r2d3 -y -c conda-forge \
    python=3.10 numpy scipy matplotlib pyyaml lxml trimesh ipython pip pytest
```

The directory-gated auto-activation hook in `~/.bashrc` (added by the initial setup) activates this env automatically when `cd`-ing into `r2d3_isaac/`.

## 3. Build the ROS 2 packages

```bash
cd ~/r2d3_isaac
bash scripts/build_packages.sh
```

This builds `rm_ros_interfaces` + `r2d3_task_interfaces` + `r2d3_model_interfaces` + `r2d3_model` into `install/`. The wrapper script sets the CMake Python-finder hints that RoboStack + recent CMake require:

```cmake
-DPython_ROOT_DIR=<ros_humble env>
-DPython_EXECUTABLE=<ros_humble env>/bin/python3
-DPython_FIND_VIRTUALENV=ONLY
-DPython_FIND_STRATEGY=LOCATION
```

To build a single package: `bash scripts/build_packages.sh r2d3_model`.

## 4. Render the V1 wrapper URDF (75b + D435 + gripper)

Once the `ros_humble` env is built, `xacro` is on its `PATH`. To render the wrapper:

```bash
~/miniforge3/bin/mamba run -n ros_humble bash -c '
  source install/setup.bash
  bash isaac_sim/urdf/render.sh
'
```

The output `isaac_sim/urdf/r2d3_v1.urdf` is gitignored (regenerable any time).

## 5. URDF → USD

```bash
scripts/urdf_to_usd.py
```

Wraps `python -m urdf_usd_converter` and auto-discovers `package://` refs. Defaults to the V1 wrapper URDF; falls back to the bare 75b URDF if the wrapper isn't rendered yet. Output: `isaac_sim/usd/Contents.usda` + a `Payload/` directory (~2 MB total).

To run a different script through the same Isaac+ROS2 plumbing:

```bash
scripts/isaacsim_ros2.sh path/to/your_script.py [args...]
```

This sets `OMNI_KIT_ACCEPT_EULA=YES`, exports `AMENT_PREFIX_PATH` to include both `install/` and the RoboStack prefix, sets `LD_LIBRARY_PATH`, and exec's the `isaac` env's Python. It deliberately does **not** source `ros_humble`'s `setup.bash` (that would prepend Python 3.11 paths and corrupt Isaac's 3.10/3.12 imports — see comment in dameria's recipe at `/usr1/home/dameria/isaacsim_ros2.sh`, which we modeled on).

## 6. Smoke test (read-only sanity)

```bash
scripts/check_env.sh    # nvidia-smi, docker info, /usr0 free, GPU avail, network, miniforge
```

## Riddle-specific notes (CMU lab)

- Workspace at `/usr1/home/semathew/r2d3_isaac/`.
- `/usr0` (where Docker's data-root lives) is chronically near-full — irrelevant to this stack since everything is under `/usr1`.
- GUI when interactive viewing is needed: TurboVNC + VirtualGL via `scripts/start_vnc.sh`. The `urdf_to_usd.py` pipeline doesn't need a display.
- Default GPU pin: `CUDA_VISIBLE_DEVICES=1` to stay clear of other users' GPU 0 workloads.

## Containerized fallback

The Docker-based path that was previously planned under `Docker/isaac/` has been removed in favor of this pip-based deployment. If you specifically need a portable image (for hackathon submission or CI), you can recreate one by composing the same isaac + ros_humble envs into a multi-stage Dockerfile — but it is no longer the primary path.
