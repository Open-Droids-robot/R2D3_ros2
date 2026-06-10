# R2D3 SDK — API reference

The `R2D3` class is the one handle you need to drive the simulated robot in
Python (in-process, no ROS required). It boots Isaac Sim, loads the robot, and
exposes a labelled control + sensing surface. Build RL, VLM, or teleop on top.

```python
from isaac_sim.r2d3_sim import R2D3

with R2D3(end_effector="dexterous") as sim:      # boots Isaac + loads the robot
    sim.reset()                                  # home pose, settle, first obs
    sim.set_arm_pose("left", [0.45, -0.21, 0.55], sim.top_down_quat)  # IK
    sim.set_gripper("left", 1.0)                 # 0 = open, 1 = closed
    rgb = sim.get_image("head")                  # numpy uint8 HxWx3, in-process
    obs = sim.step(n=30)                          # advance physics, get an Observation
```

Run any script through the launcher (sets env + library paths):
```bash
scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
```

## Construction

`R2D3(*, end_effector="dexterous", headless=True, mobile=False, usd_path=None,
enable_cameras=True, enable_ros=False, camera_resolution=(640,480),
stage_units_m=1.0, setup=None)`

| arg | meaning |
|---|---|
| `end_effector` | `"dexterous"` (Inspire 5-finger hand) or `"gripper"` (2-finger). Picks the USD + joints. |
| `headless` | run without the Kit window. |
| `mobile` | load the wheels-revolute build + free the base so it can be driven (`set_base_pose`). |
| `usd_path` | override the auto-selected USD. |
| `enable_cameras` | attach in-process camera render products (needed for `get_image`). |
| `enable_ros` | also publish/subscribe on `/r2d3/sim/*` (the bridge/eval path). |
| `setup` | `setup(world)` callable run after the robot loads but **before** `world.reset()` — add task objects (tables, cubes) here. See `examples/07_grasp_cube.py`. |

## Lifecycle

| method | description |
|---|---|
| `reset() -> Observation` | go to the home pose, settle, warm up the renderer, return the first observation. |
| `step(*, render=True, n=1) -> Observation` | advance `n` physics steps (flush ROS commands if enabled), return a fresh observation. |
| `close()` | shut down ROS (if any) + the Isaac app. |
| `with R2D3(...) as sim:` | context-manager; `close()` on exit. |

## Control

| method | description |
|---|---|
| `set_arm_joints(side, q)` | set the 7 arm-joint position targets (rad). `side` ∈ {"left","right"}. |
| `set_arm_pose(side, position, quat_wxyz=None, *, pos_tol, ori_tol) -> bool` | IK to a Cartesian EE pose (world frame); applies if solved. **Left arm only** (right raises). |
| `set_head(pan, tilt)` | head pan/tilt (rad). |
| `set_lift(height_m)` | body lift height (m, 0..1). |
| `set_gripper(side, frac)` | end-effector-agnostic open/close: `frac` 0 = open, 1 = closed (works for both EEs). |
| `set_joint_targets(mapping)` | set arbitrary joint targets by name (`{name: value}`) — e.g. spinning wheels. |
| `set_base_pose(position, quat_wxyz)` | kinematically place the mobile base (needs `mobile=True`). |
| `go_home()` | drive to the canonical home pose. |
| `top_down_quat` (property) | orientation (wxyz) for a top-down grasp. |

## Sensing (all numpy, in-process)

| method | returns |
|---|---|
| `get_image(camera="head", *, depth=False)` | RGB `uint8 HxWx3`; `(rgb, depth)` if `depth=True` (depth = `float32 HxW`, metres). Cameras: `"head"`, `"l_wrist"`, `"r_wrist"`. |
| `get_images(depth=False)` | `{camera: rgb}` for all cameras. |
| `get_joint_positions()` | `{joint_name: rad|m}`. |
| `get_joint_state()` | `JointState(names, positions)`. |
| `get_lift()` | lift height (m). |
| `get_wrench(side)` | wrist 6-axis FT `[fx,fy,fz,tx,ty,tz]` (zeros in free space). |
| `get_ee_pose(side)` | FK end-effector pose `(xyz, quat_wxyz)`. |
| `get_observation()` | `Observation` bundling images + depths + joints + lift + wrenches + EE poses. |

`Observation` and `JointState` are dataclasses (`from isaac_sim.r2d3_sim import Observation, JointState`).

## Escape hatches

`sim.robot` (the `Robot` wrapper), `sim.world` (the Isaac `World`), `sim.ik`
(`ArmIK`), `sim.cameras` (`CameraRig`) are public — drop down when you need the
low-level API.

## Notes / limits

- **End-effector** is fixed per process (`R2D3_EE` is read at import). Construct a
  new `R2D3` to switch; the grasp/IK adapt automatically (per-EE URDF + Lula yaml).
- **IK is left-arm only** — the upstream Lula descriptions are `l_joint1..7`;
  `set_arm_pose("right", ...)` raises until a mirrored right-arm yaml exists.
- **Cameras** need `enable_cameras=True`; the first `get_image` warms up RTX (~12
  render steps). Frames are real RGB/depth; for nicer contrast add a dark
  backdrop + tune lights (`helpers.set_lighting`).
- **Base** is kinematic (`set_base_pose`) in V1 — wheel-physics driving is V2.
  Spin the wheels visually with `set_joint_targets` (see `examples/06_drive_base.py`).
- **Gravity is off** on the robot articulation (stable position drives); wrench
  reads contact/holding forces, not static gravity load.
