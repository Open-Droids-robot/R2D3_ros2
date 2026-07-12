# r2d3_mujoco — MuJoCo Simulation Backend for R2D3

**Date:** 2026-07-11
**Status:** Approved design, pending implementation
**Goal:** Full-parity MuJoCo simulation backend mirroring the existing Gazebo Harmonic
support (`dual_rm_simulation`), as a self-contained package `r2d3_mujoco` at the repo
top level. Must work on ROS 2 Jazzy **and** Humble.

## Context

The repo simulates the R2D3 dual-arm mobile robot (RealMan 65b/75b arms, diff-drive AGV
base, platform lift, head, lidar, IMU, RGB-D camera) in Gazebo Harmonic via
`dual_rm_simulation` + `gz_ros2_control` + `ros_gz_bridge`. Nav2/SLAM and MoveIt2 stacks
consume `/scan`, `/imu`, `/camera/*`, `/clock`, and ros2_control controllers.

MuJoCo support is built on **`mujoco_ros2_control`** (ros-controls; installed as the
Jazzy binary **0.0.3** at `/opt/ros/jazzy`). It provides:

- A ros2_control `SystemInterface` running MuJoCo physics in-process (custom
  `ros2_control_node` executable) — same controller YAMLs work.
- Built-in `/clock` publishing (no bridge needed).
- Built-in lidar (rangefinder array → `LaserScan`) and RGB-D camera (offscreen render →
  image/depth/camera_info) publishing, configured through `<sensor>` entries in the
  URDF `ros2_control` block (0.0.3 API; upstream `main` moved these into
  `mujoco_ros2_control_plugins` — see Version compatibility).
- IMU/FTS mapping from MuJoCo `framequat`/`gyro`/`accelerometer` sensors to ros2_control
  sensor state interfaces (consumed by `imu_sensor_broadcaster`).
- URDF→MJCF conversion tooling (`robot_description_to_mjcf.sh` /
  `make_mjcf_from_robot_description.py`) with `<mujoco_inputs>` support
  (`raw_inputs`, `processed_inputs`: `camera`, `lidar`, `modify_element`,
  `decompose_mesh`), `--add_free_joint` for mobile bases, and `--scene`.
  The script self-bootstraps its Python deps into a venv at `$ROS_HOME/ros2_control/.venv`.
- Free-joint odometry publishing (`odom_free_joint_name` → `nav_msgs/Odometry`),
  pause/reset/step services.

## Decisions made during brainstorming

1. **Scope:** full parity with Gazebo — controllers, IMU, lidar, RGB-D camera, Nav2/SLAM
   (all three backends), MoveIt2.
2. **Integration:** self-contained `r2d3_mujoco` package; existing packages untouched.
   The MuJoCo bringup *includes* existing sub-launches/configs (from
   `dual_rm_navigation`, MoveIt config) so only top-level wiring is duplicated.
3. **Location:** `R2D3_ros2/r2d3_mujoco/` (top level).
4. **MJCF strategy:** cached runtime conversion — compile once, reuse converted meshes
   and MJCF; recompile automatically at launch when the source URDF (or inputs) change,
   detected by hashing the xacro output.
5. **Distros:** Jazzy (primary, binary installed) and Humble (source build of
   `mujoco_ros2_control`; launch files distro-aware).

## Package layout

```
r2d3_mujoco/
├── CMakeLists.txt              # ament_cmake, install-only (config/launch/urdf/worlds/scripts)
├── package.xml
├── scripts/
│   └── ensure_mjcf.py          # hash-check → cached MJCF or trigger conversion; publishes MJCF
├── config/
│   ├── controllers_65b.yaml    # copy of Gazebo controllers + imu_sensor_broadcaster
│   ├── controllers_75b.yaml
│   └── (rviz reused from dual_rm_navigation / r2d3_bringup)
├── launch/
│   ├── mujoco_sim.launch.py    # mirror of gz_sim.launch.py
│   └── bringup_sim.launch.py   # sim + Nav2/SLAM + optional MoveIt + RViz
├── urdf/
│   ├── r2d3_mujoco.urdf.xacro  # mirror of r2d3_sim.urdf.xacro (arm_model 65b/75b)
│   ├── mujoco_inputs.urdf.xacro       # <mujoco_inputs>: actuators, sensors, camera, lidar
│   └── ros2_control/mujoco_ros2_control.urdf.xacro
└── worlds/
    └── nav_empty.xml           # MuJoCo scene mirroring worlds/nav_empty.sdf
docs: simulation_quickstart_mujoco.md at repo root (mirrors simulation_quickstart_gz.md)
```

No compiled code — xacro + config + launch + one Python helper script.

## Robot model (URDF → MJCF)

`r2d3_mujoco.urdf.xacro` mirrors `r2d3_sim.urdf.xacro`: `base_footprint`, the shared
`dual_rm_description` mechanical model, and MuJoCo-specific includes instead of the
Gazebo ones:

- **`mujoco_inputs.urdf.xacro`** — emits the `<mujoco_inputs>` block:
  - `raw_inputs`:
    - `<option integrator="implicitfast"/>` (recommended modern default; joint damping
      integrated implicitly → stable arms at 500 Hz physics). Add
      `noslip_iterations` only if lateral wheel drift proves problematic (MuJoCo soft
      contacts are slightly slippery by design).
    - `<actuator>` block:
      - Wheels: `<velocity joint="joint_left_wheel" kv=.../>` ×2 (native `velocity`
        command interface support).
      - Arms (6 or 7 DOF ×2), platform, head: `<position kp=... dampratio="1.0"
        ctrlrange=.../>` per joint (native `position` support). `kp` starting points
        from upstream demos (25000 arms; tune platform higher for payload).
      - Caster swivel joints: **no actuators** (passive, state-only — matches the
        existing passive `ros2_control` joint entries).
    - `<sensor>`: IMU triple at an `imu` site —
      `framequat name="imu_sensor_quat"`, `gyro name="imu_sensor_gyro"`,
      `accelerometer name="imu_sensor_accel"` (upstream naming convention).
  - `processed_inputs`:
    - `<camera site="camera_optical_frame" name="camera" mode="fixed"
      resolution="640 480" fovy="46.8"/>` — fovy computed from Gazebo's 1.047 rad
      horizontal FOV at 640×480.
    - `<lidar ref_site="lidar_link" sensor_name="rf" min_angle="-2.0943951"
      max_angle="2.0943951" angle_increment="${(max-min)/(240-1)}"/>` (≈0.017526 rad) —
      240 rangefinders, matching the Gazebo lidar (±120°, 240 samples).
    - `<modify_element>` entries as needed for wheel/caster physics: friction and
      `condim` on drive-wheel geoms; low-friction contacts for caster wheels
      (mirrors the Gazebo fix where drive wheels got primitive cylinder collisions).
- **`mujoco_ros2_control.urdf.xacro`** — same joint/command/state interface list as the
  Gazebo one (identical initial values), but:
  - `<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>`
  - `<param name="mujoco_model_topic">/mujoco_robot_description</param>` (MJCF from topic)
  - `<param name="headless">` from launch arg (default false — MuJoCo's viewer is cheap,
    unlike the Gazebo GUI; headless available for CI)
  - `<param name="odom_free_joint_name">` → publishes ground-truth base odometry on
    `/simulator/ground_truth_odom` (bonus over Gazebo; `diff_drive_controller` still
    provides the real `/odom` + TF from wheel encoders, unchanged)
  - `<sensor name="imu_sensor">` with `mujoco_type: imu` mapping (consumed by
    `imu_sensor_broadcaster` → `/imu`)
  - `<sensor name="camera">` params: `frame_name: camera_optical_frame`,
    `image_topic: /camera/image`, `depth_topic: /camera/depth_image`,
    `info_topic: /camera/camera_info` (matches Gazebo topic names)
  - `<sensor name="lidar">` params: `frame_name: lidar_link`, angles/increment as above,
    `range_min: 0.55`, `range_max: 12.0`, `laserscan_topic: /scan`
    (`range_min: 0.55` preserved — it exists to prevent lidar self-detection, which
    feeds SLAM/costmap configs)

**Floating base:** the converter is invoked with `--add_free_joint`, giving the robot a
MuJoCo free joint at the root so the diff-drive base can move. Wheels/casters interact
with the scene ground plane through contacts.

## MJCF caching pipeline (`ensure_mjcf.py`)

Runs as a node in `mujoco_sim.launch.py`, replacing a bare converter call:

1. Receives the xacro-generated URDF string (same `Command` substitution as Gazebo launch).
2. Computes SHA-256 over: URDF string + scene file content + converter-relevant args.
   Hashing the *xacro output* catches changes anywhere in the include chain
   (`dual_rm_description`, sensors, inputs).
3. Cache dir: `$ROS_HOME/r2d3_mujoco/<robot_model>/` (beside the converter's venv),
   containing `model.xml`, `assets/` (converted OBJ meshes — the expensive part),
   and `checksum`.
4. Hash match → publish cached MJCF on `/mujoco_robot_description`
   (transient-local/latched) and stay alive; **no conversion**.
5. Mismatch/missing → invoke `robot_description_to_mjcf.sh` with `--save_only
   --add_free_joint -m <inputs> --scene <world> -o <cache_dir>`, write new checksum,
   publish. First run also bootstraps the converter venv (one-time, minutes).
6. `force_recompile:=true` launch arg bypasses the hash check.

The MuJoCo `ros2_control_node` reads the MJCF from the topic and cannot tell cached
from fresh.

## Launch files

### `mujoco_sim.launch.py` (mirror of `gz_sim.launch.py`)

Args: `robot_model` (65b|75b, default 65b), `world` (default `worlds/nav_empty.xml`),
`headless` (default false), `force_recompile` (default false).

Nodes: `ensure_mjcf.py` → `robot_state_publisher` (use_sim_time) →
`mujoco_ros2_control/ros2_control_node` (controllers YAML; Humble-conditional
`~/robot_description` remap — upstream demo pattern) → spawners with the same event
sequencing as Gazebo (JSB first, then diff_drive/left_arm/right_arm/platform +
`imu_sensor_broadcaster`) → `depth_image_proc` point-cloud node producing
`/camera/points` from depth + camera_info.

No `ros_gz_bridge`, no spawn-entity: the robot is part of the MJCF and `/clock` comes
from the MuJoCo node.

### `bringup_sim.launch.py`

Mirrors `dual_rm_navigation/bringup_sim.launch.py` + `r2d3_bringup` MoveIt wiring, but
includes `mujoco_sim.launch.py` instead of the Gazebo launch. Same args: `robot_model`,
`world`, `mode` (slam|localization), `slam_type` (slam_toolbox|rtabmap|rtabmap_depth_only),
`map`, `use_rviz`, `use_moveit`. Reuses (by include/path) `dual_rm_navigation`'s
`slam.launch.py`, `rtabmap*.launch.py`, `localization.launch.py`, `navigation.launch.py`,
its param YAMLs and RViz configs, and the existing MoveIt launch. Same 10 s
`TimerAction` staging.

## Controllers

`controllers_{65b,75b}.yaml` = copy of the Gazebo files plus:

```yaml
imu_sensor_broadcaster:
  type: imu_sensor_broadcaster/IMUSensorBroadcaster
  # sensor_name: imu_sensor, frame_id: imu_link, topic: /imu
```

`diff_drive_controller` params already Humble-compatible (`use_stamped_vel: true`).

## Topic parity table

| Topic | Gazebo source | MuJoCo source |
|---|---|---|
| `/clock` | ros_gz_bridge | mujoco ros2_control_node (built-in) |
| `/scan` | gpu_lidar + bridge | rangefinder array (built-in LaserScan) |
| `/imu` | imu sensor + bridge | MJCF sensors + imu_sensor_broadcaster |
| `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | rgbd_camera + bridge | built-in offscreen camera render |
| `/camera/points` | rgbd_camera + bridge | depth_image_proc from depth+info |
| `/joint_states`, `/odom`+TF, arm/platform actions | ros2_control | identical (same controllers) |

## World

`worlds/nav_empty.xml`: MuJoCo scene with ground plane, lighting, skybox (from the
upstream scene template) plus the same obstacle set as `nav_empty.sdf` translated to
MJCF geoms, so SLAM has features to map. Scene is merged at conversion time via
`--scene` (and is part of the cache hash).

## Humble + Jazzy compatibility

- Launch files use the upstream pattern: `ROS_DISTRO`-conditional remapping of
  `~/robot_description`; no Jazzy-only launch API.
- Jazzy: binary `mujoco_ros2_control` 0.0.3 (installed). Humble: build
  `mujoco_ros2_control` from source (documented in quickstart), **pinned to the 0.0.3
  release tag** so the sensor API matches this package's URDF conventions.
- **Version compatibility note:** upstream `main` moved camera/lidar publishing from the
  base interface into `mujoco_ros2_control_plugins`. This package targets the 0.0.3 API
  (sensors declared in the `ros2_control` block). When distros ship a newer version,
  the URDF sensor declarations stay, but a plugins-config file may need to be added —
  called out in the quickstart's troubleshooting.

## MuJoCo best practices applied (from research)

- `implicitfast` integrator (upstream-recommended default; stable with joint damping).
- Soft contacts are intentionally slippery laterally → plan: tune wheel geom
  `friction`/`condim` via `modify_element`; enable `noslip_iterations` only if odometry
  drift vs ground truth is excessive.
- Convex-hull collision: robot meshes get convex hulls automatically. Drive wheels may
  need primitive collision geoms (same class of problem already hit in Gazebo);
  `modify_element`/raw geom overrides handle it.
- 240 rangefinders is a moderate CPU cost per physics step; mitigation if slow: reduce
  sample count via xacro property (SLAM tolerates fewer beams) — parity spec kept as
  default.
- Gripper-style mimic joints (if 2-finger gripper returns later): tendon + equality
  constraint pattern per upstream docs.

## Risks & validation order

1. **Floating base + wheel physics (highest risk):** validate first with a spike —
   convert the URDF with `--add_free_joint`, load in `mujoco_vendor simulate`, drive
   wheels manually. Casters (8 passive joints) may over-constrain contacts; fallback is
   low-friction sphere geoms for casters (state interfaces stay, physics simplified).
2. **Converter robustness:** upstream calls the tool "highly experimental". The R2D3
   URDF has many STL meshes; mesh conversion issues are possible. Mitigation:
   `decompose_mesh`/`modify_element` hooks, and the cache means we only pay/debug
   conversion when the model changes.
3. **Camera rendering headless/EGL:** offscreen GLFW rendering may hit the same GPU
   driver quirks seen with Gazebo; document fallback env vars in troubleshooting.
4. **Initial joint positions:** `initial_value` params should carry over (same
   mechanism); verify arms start in the same tucked pose as Gazebo.

## Testing / verification

- `colcon build` + `ros2 launch r2d3_mujoco mujoco_sim.launch.py` (65b and 75b).
- `ros2 topic hz` on `/clock`, `/scan`, `/imu`, `/camera/image`, `/camera/depth_image`,
  `/camera/points`, `/joint_states`.
- Teleop drive + `r2d3_test_nodes` (test_agv_motion, test_arm_motion) pass unchanged.
- Nav2 SLAM smoke test: `bringup_sim.launch.py`, drive around, verify map builds and a
  Nav2 goal executes.
- MoveIt: plan + execute on `left_arm` via RViz MotionPlanning.
- Cache behaviour: second launch skips conversion; touching a description xacro
  triggers recompile; `force_recompile:=true` works.

## Out of scope

- Modifying any existing package (`dual_rm_simulation`, `dual_rm_navigation`,
  `r2d3_bringup` remain untouched).
- GPU-accelerated lidar, MJX, multi-robot.
- 2-finger gripper (currently reverted on `main`).
