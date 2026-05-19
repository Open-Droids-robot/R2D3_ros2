# Setup

> Reproduces the development environment on a fresh machine (Ubuntu 22.04 or 24.04 host with NVIDIA GPU + driver ≥ 535).

## Host prerequisites
- NVIDIA driver ≥ 535 (`nvidia-smi` works)
- Docker ≥ 24 with the `nvidia-container-toolkit` runtime configured
- User in `docker` group
- Optional: TurboVNC + VirtualGL for remote GUI on a headless box

## Clone

```bash
git clone https://github.com/Open-Droids-robot/R2D3_ros2.git r2d3_isaac
cd r2d3_isaac
git checkout isaac-sim-v1
```

## Python environment (host-side)

We use Miniforge (mamba) for the host environment. ROS2 and Isaac Sim run inside Docker — the host env is only for URDF tooling, eval scripting, and notebooks.

```bash
# Install Miniforge to your home dir (NOT system-wide)
wget -qO miniforge.sh \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash miniforge.sh -b -p "$HOME/miniforge3"
~/miniforge3/bin/mamba create -y -n r2d3 python=3.10 numpy scipy matplotlib pyyaml lxml trimesh
```

Then activate inside the repo:

```bash
eval "$($HOME/miniforge3/bin/conda shell.bash hook)"
mamba activate r2d3
```

## Docker stack
TODO: Once `Docker/isaac/` is populated, document `compose.isaac.yaml` here.

## NGC login (required for Isaac Sim image)

```bash
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key from https://ngc.nvidia.com>
```

## Riddle-specific notes (CMU lab)
- Workspace at `/usr1/home/semathew/r2d3_isaac/`
- Docker data-root on `/usr0` is chronically full — don't pull large images until admin confirms cleanup
- GUI via TurboVNC at `/opt/TurboVNC/bin/vncserver`, GL acceleration via `vglrun`
- Default GPU pin: `CUDA_VISIBLE_DEVICES=1` (host) / `--gpus '"device=1"'` (Docker)
