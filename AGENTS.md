# AGENTS.md — working guide for AI coding agents

This file tells an AI agent everything needed to work in this repo end-to-end:
how to run things, control the robot, swap parts, build assets, verify changes,
and where everything lives. Read it before editing.

---

## 1. What this repo is

Two layers share one repo:

1. **Isaac Sim platform** (the active work, branch `isaac-sim-v1`) — `isaac_sim/`,
   `scripts/`, `docs/`. A Python SDK that boots NVIDIA Isaac Sim 6.0, loads the
   R2D3 robot, and exposes in-process control + sensing. **Default to this.**
2. **Upstream ROS 2 / real-robot stack** — `ros2_*/`, `r2d3_model*/`,
   `rm_ros_interfaces` (under `ros2_rm_robot/`), `r2d3_humble_bridge/`. The real
   robot's drivers + the sim↔real ROS bridge. Documented in
   [`docs/ros2_packages.md`](docs/ros2_packages.md). Touch only for ROS-path work.

R2D3 is a **dual-arm composite-lifting mobile manipulator** (two 7-DOF arms on a
vertical lift + a wheeled base) — not a legged humanoid.

---

## 2. Golden rules

- **Never run the isaac env's Python directly.** Launch every script through the
  wrapper, which sets env vars, EULA, and library paths:
  ```bash
  scripts/isaacsim_ros2.sh <script.py> [args...]
  ```
- **`SimulationApp` boots once per process, before any `omni`/`isaacsim` import.**
  The `R2D3` facade and `boot.py` handle this — don't import `omni`/`isaacsim` at
  module top level.
- **The end-effector is locked at first import** of any `r2d3_sim` submodule that
  reads `EE_TYPE` (e.g. `sim_topics`). Construct `R2D3(end_effector=...)` (or set
  `R2D3_EE`) **before** importing those submodules. `R2D3.__init__` raises if you
  get this wrong (see §8).
- **`print(..., flush=True)`** in scripts — Isaac hard-exits on shutdown and drops
  buffered stdout. The SDK flushes in `close()`, but bare prints can be lost.
- **GPU**: on the dev box the launcher pins `CUDA_VISIBLE_DEVICES=1`. Needs an
  NVIDIA GPU; there is no CPU fallback.
- **Commit, don't push**, unless the user explicitly says to push.

---

## 3. Setup

One-time, idempotent:
```bash
bash scripts/bootstrap.sh        # creates the `isaac` conda env, installs Isaac
                                 # Sim 6.0 + `pip install -e .`, runs the smoke test
```
Conda paths auto-detect; override with `R2D3_CONDA_BASE` / `R2D3_ISAAC_ENV` /
`R2D3_ISAAC_PY`. The `ros_humble` env is **optional** (ROS bridge only). Details:
[`docs/setup.md`](docs/setup.md), [`docs/run.md`](docs/run.md).

---

## 4. Controlling the robot — the SDK

`from isaac_sim.r2d3_sim import R2D3`. One object boots, controls, and senses.

```python
with R2D3(end_effector="dexterous", headless=True) as sim:
    sim.reset()                 # home pose, settle, warm up renderer
    sim.step(n=30)              # advance physics (returns an Observation)
```

**Control** (sides are `"left"`/`"right"`):

| Method | Effect |
|---|---|
| `set_arm_joints(side, q)` | 7 arm joint targets (rad) |
| `set_arm_pose(side, xyz, quat_wxyz=None)` | **IK** to a Cartesian pose → bool solved. **Left arm only.** |
| `set_head(pan, tilt)` | head (rad) |
| `set_lift(height_m)` | body lift (0–1 m) |
| `set_gripper(side, frac)` | EE-agnostic open/close, `frac` 0=open … 1=closed |
| `set_joint_targets({name: val})` | arbitrary joints (e.g. wheels) |
| `set_base_pose(xyz, quat_wxyz)` | kinematic base placement (needs `mobile=True`) |
| `go_home()` / `sim.top_down_quat` | home pose / top-down grasp orientation |

**Sensing** (all numpy, in-process):

| Method | Returns |
|---|---|
| `get_image(cam="head", depth=False)` | RGB `uint8 HxWx3` (or `(rgb, depth)`). Cams: `head`, `l_wrist`, `r_wrist` |
| `get_wrench(side)` | wrist 6-axis `[fx,fy,fz,tx,ty,tz]` |
| `get_ee_pose(side)` | FK `(xyz, quat_wxyz)` |
| `get_joint_state()` / `get_joint_positions()` | all DOFs |
| `get_observation()` | bundles images+depths+joints+lift+wrenches+EE poses |

**Escape hatches** (public attrs): `sim.robot` (low-level `Robot`), `sim.world`
(Isaac `World`), `sim.ik` (`ArmIK`), `sim.cameras` (`CameraRig`).

Full reference: [`docs/api.md`](docs/api.md). Higher-level interfaces in
`isaac_sim/r2d3_sim/envs/`: `R2D3Env` (gymnasium RL), `PerceptionLoop` (VLM),
`TeleopServer` — see [`docs/platform.md`](docs/platform.md).

---

## 5. Switching parts

**End-effector** — `R2D3(end_effector="dexterous" | "gripper")` (or set `R2D3_EE`).
Picks the USD, the joint set (`sim_topics.LEFT_HAND_JOINTS` etc.), and the IK URDF
+ Lula yaml (`ik._CFG`). Fixed per process (see the import-lock rule, §8). Examples
accept `--ee dexterous|gripper`. The USD is chosen by `(end_effector, mobile)`:

| | `mobile=False` | `mobile=True` |
|---|---|---|
| `dexterous` | `usd_dexterous/` | `usd_mobile/` |
| `gripper`   | `usd_gripper/`   | `usd_gripper_mobile/` |

The legacy RM `*_hand_base_link` hand-flange mesh is hidden for **both** EEs in
`scene._hide_legacy_hand_flanges()` — the dexterous hand and the gripper jaws both
mount on the `l_hand_link` frame, not that mesh, so leaving it visible read as a
redundant second hand.

**Mobile base** — `R2D3(mobile=True)` loads the per-EE mobile build (AGV wheels
revolute) and disables the `root_joint` so the base is free; then `set_base_pose(...)`
moves it and `set_joint_targets({"joint_left_wheel": ...})` spins the wheels. The
static build pins the base.

**Task objects** — pass `setup=fn` to `R2D3(...)`; `fn(world)` runs after the robot
loads but before `world.reset()`. Add `isaacsim.core.api.objects` there (see
`isaac_sim/examples/07_grasp_cube.py`).

---

## 6. Building / regenerating assets

The built USD assets ship in the repo. To regenerate (needs the `ros_humble` env
for `xacro`):
```bash
bash scripts/build_robot.sh dexterous|gripper|mobile|gripper-mobile|both|all
```
Pipeline: `urdf/r2d3_v1.urdf.xacro` → `render.sh <ee> <weld>` (flat URDF, in
`ros_humble`; `weld=0` keeps the AGV wheels revolute for the mobile builds) →
`scripts/urdf_to_usd.py` (USD, in `isaac`) → `usd_<out>/r2d3_v1.usda`.

The Lula IK URDFs are **mesh-free** (`scripts/make_lula_urdf.py` strips `<mesh>` →
`<box>`; the flat URDFs have absolute mesh paths and are gitignored). `ik.py`
loads `urdf/r2d3_v1_<ee>_lula.urdf`.

---

## 7. Repo map — where to put things

```
isaac_sim/r2d3_sim/
  r2d3.py        R2D3 facade (control + sensing + lifecycle)        ← main surface
  robot.py       articulation wrapper (drives, FT, FK)
  scene.py       USD load + ground/lights + add_visual_box/world_range
  sensors.py     camera prims + OmniGraph ROS publishers
  cameras.py     CameraRig (in-process RGB/depth numpy)
  ik.py          ArmIK (Lula, left arm; per-EE URDF/yaml in _CFG)
  scenes.py      training environments (warehouse/kitchen/living_room) + objects
  perception.py  open-vocabulary detection (OWL-ViT) via subprocess; head-cam -> boxes
  boot.py        SimulationApp launch + ROS-ext enable
  helpers.py     quats, prim lookup, world pose, RGBA, lighting, GIF
  sim_topics.py  joint-name + topic-name contract; reads R2D3_EE → EE_TYPE
  sim_adapter.py rclpy node (ROS state/command surface)
  bring_up.py    ROS entry point (the bridge path)
  envs/          rl_env.py, vlm_loop.py, teleop.py
isaac_sim/examples/   01–09 demos (08 = pick a mug; 09 = OWL-ViT-perception clear-island pick/place)
isaac_sim/tests/      smoke_sdk.py, diag_all_joints.py, diag_motion_gif.py,
                      diag_scenes.py (scene render + --check); grasp_lift_ik/move_task
                      (GIF demos); diagnostics/ = archive
scripts/              bootstrap.sh, isaacsim_ros2.sh, build_robot.sh,
                      urdf_to_usd.py, make_lula_urdf.py, _conda_env.sh
docs/                 setup, run, api, platform, examples, bridge, scenes, architecture, ros2_packages
```
Add a new example → `isaac_sim/examples/NN_name.py` (use the SDK).
Add a new env/interface → `isaac_sim/r2d3_sim/envs/`. New control/sensing → extend
`r2d3.py` (and `robot.py` if it needs the articulation). New training scene or
object → `scenes.py` (`add_fixed_box` for surfaces, `add_object` for manipulables).

---

## 8. Gotchas & footguns

- **EE import-lock**: importing `sim_topics`/`scene`/`robot` before `R2D3(...)`
  caches the default end-effector; then `R2D3(end_effector="gripper")` would load
  the wrong robot. `R2D3.__init__` raises on the mismatch. In scripts, import only
  `R2D3` (+ pure helpers) at top; import `sim_topics` *inside* `main()` after
  constructing `R2D3` (see `tests/diag_all_joints.py`).
- **Black renders**: RTX needs ~12–20 warm-up render steps; `reset()` / first
  `get_image()` handle it. The white robot needs a dark/contrast backdrop + lights
  to be visible (`helpers.set_lighting`, `scene.add_visual_box`) — see
  `tests/diag_motion_gif.py`.
- **Stale camera frames**: `CameraRig` warms up **once**; after that `get_image()`
  returns the last *rendered* frame. If you change the robot/head pose and step with
  `render=False`, the captured frame is stale — step with `render=True` after posing,
  before capturing. See `09_kitchen_clear_island.py` / `tests/diag_perception.py`.
- **ML models crash the kit process**: importing `transformers`/big `torch` models
  in-process segfaults the Isaac kit app (threading/CUDA-context conflict, no
  traceback). Run them in a **subprocess** — `r2d3_sim.perception.detect()` shells
  out to a clean Python and returns JSON. Also: a COCO detector (torchvision) does
  NOT recognise the sim YCB props (a red mug → "chair"); use **open-vocabulary**
  OWL-ViT (text query "a red mug"), which transfers to the sim render.
- **Perception → grasp**: head-cam pixel + radial depth (`distance_to_camera`) +
  the head `Camera` prim's intrinsics/world-pose unproject to a world point (≈2.5 cm
  accurate). Observe from ~0.3 m back with the arm stowed (the recessed head lens is
  occluded by the robot's own body/arms at the grasp pose).
- **`rep` camera `look_at`** fails for near-level views — use a downward angle.
- **Gravity is off** on the robot articulation (stable position drives); `get_wrench`
  reads contact/holding forces, not static gravity load.
- **IK is left-arm only** (Lula descriptions are `l_joint1..7`); `set_arm_pose("right")`
  raises.
- **Base is kinematic** in V1 (`set_base_pose`); spin wheels visually with
  `set_joint_targets`. Wheel-physics driving is V2. A free mobile base (gravity off)
  has no friction, so **re-pin it each step** (`set_base_pose` in the loop) when
  holding a pose — else it drifts. See `tests/diag_scenes.py`.
- **IK uses the base orientation**: `set_arm_pose` syncs the real base pose, so a
  rotated robot (e.g. facing -X at a counter) solves correctly. The wrist offset for
  a top-down grasp is toward the robot — opposite sign to a +X-facing robot.
- **Scene surfaces are collidable, room floors are visual-only**: objects rest on
  `add_fixed_box` counters/island/table (or the warehouse floor); the room
  floor/walls have no collider (a collidable ground would perturb the held base).
- **Grasping small objects**: friction alone is unreliable — the grasp examples
  attach the object with a fixed joint at contact, and **freeze the object on the
  table during the approach** so the weld is computed from a clean pose (else it
  snaps). See `examples/07_grasp_cube.py`.
- **Don't edit collision-mesh prims mid-sim** ("Simulation view object is
  invalidated") — use the physics API (drives / `set_joint_targets`).

---

## 9. Verifying changes

```bash
scripts/isaacsim_ros2.sh isaac_sim/tests/smoke_sdk.py                      # SDK boots, controls, senses
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_all_joints.py --ee dexterous # every DOF moves (also --ee gripper, --mobile)
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_motion_gif.py --ee dexterous # third-person motion GIF (visual check)
scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene kitchen --check --no-render  # objects settle + robot stable
```
`diag_all_joints.py` prints PASS/FAIL per joint and exits non-zero on failure —
use it after any control/asset change. Expected: dexterous 41 DOF, gripper 21,
mobile 51, all PASS. Isaac runs are slow (~1–5 min); run headless, in the
background, and poll the log.

---

## 10. The ROS bridge (only if asked)

`R2D3(enable_ros=True)` runs the rclpy adapter in-process (publishes `/r2d3/sim/*`).
The full real-robot path is two processes (Isaac speaks std_msgs; `r2d3_humble_bridge`
translates the custom `rm_ros_interfaces`). See [`docs/bridge.md`](docs/bridge.md).
This needs the `ros_humble` env; the in-process SDK does not.
