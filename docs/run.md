# Running R2D3 in Isaac Sim

There are two ways to drive the robot. Pick based on what you're building.

| | **Python SDK** (`R2D3`) | **ROS bridge** (`bring_up.py`) |
|---|---|---|
| Use for | RL, VLM, scripted control, data gen | the eval harness, MoveIt2, the real-robot model stack |
| Surface | in-process numpy (`get_image`, `set_arm_pose`, …) | `/r2d3/sim/*` + `/r2d3/observations` topics |
| Process | one | two (Isaac + `r2d3_humble_bridge`) |
| Start here | **yes** | see [bridge.md](bridge.md) |

Everything launches through the wrapper, which sets the env vars + library paths
(and, for ROS, the bundled rclpy / RMW):

```bash
scripts/isaacsim_ros2.sh <script.py> [args...]
```

> On Riddle, the launcher pins `CUDA_VISIBLE_DEVICES=1` by default to stay off
> other users' GPU 0.

## Quickstart (SDK)

```bash
# the simplest control + sensing demo
scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py

# grab the cameras to PNGs
scripts/isaacsim_ros2.sh isaac_sim/examples/02_cameras.py

# full IK grasp-and-lift of a cube
scripts/isaacsim_ros2.sh isaac_sim/examples/07_grasp_cube.py
```

See [examples.md](examples.md) for the full list and [api.md](api.md) for the SDK
reference.

## Verify the install

```bash
scripts/isaacsim_ros2.sh isaac_sim/tests/smoke_sdk.py
```

Expected: `[smoke] DONE` after lines showing `num_dof=41`, `set_arm_pose ok=True`,
three cameras returning `(480, 640, 3)` frames, a wrench, an EE pose, and a 41-joint
state. First boot is slow (asset + shader compile); subsequent runs are faster.

## Building the robot USD

The USD assets (`isaac_sim/usd_dexterous/`, `usd_gripper/`, `usd_mobile/`) are
generated from the xacro:

```bash
scripts/build_robot.sh both        # dexterous + gripper (fixed base)
# mobile (wheels revolute) build:
bash isaac_sim/urdf/render.sh dexterous 0
scripts/urdf_to_usd.py --urdf isaac_sim/urdf/r2d3_v1_dexterous_mobile.urdf --usd-dir isaac_sim/usd_mobile
```

## Switching end-effector / base

- End-effector: `R2D3(end_effector="dexterous")` or `"gripper"`. Fixed per
  process (the choice is read at import); construct a new `R2D3` to switch.
- Mobile base: `R2D3(mobile=True)` loads the wheels-revolute build and frees the
  base so `set_base_pose` works (see `examples/06_drive_base.py`).

## Troubleshooting

- **Black images** — call `reset()` (or `get_image` once) to warm up RTX; add a
  backdrop + tune lights for contrast on the white robot (`helpers.set_lighting`).
- **Script prints missing** — Isaac hard-exits on shutdown; the SDK flushes stdout
  in `close()`, but if you bypass it, add `flush=True` to your prints.
- **`isaacsim` import errors at boot** — install needs `isaacsim[all,extscache]==6.0.0.0`
  (the `[extscache]` ~5.7 GB is required); see [setup.md](setup.md).
