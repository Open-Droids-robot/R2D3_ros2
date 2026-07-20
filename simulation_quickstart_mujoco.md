# R2D3 MuJoCo Simulation – Quick Start Guide

## Prerequisites

```bash
# Build the workspace
cd ~/Ros2_Workspaces/R2D3_ros2
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Always source `install/setup.bash` in every new terminal.

### `mujoco_ros2_control`

**Jazzy:** install the pinned binary package (already present on this machine):

```bash
sudo apt install ros-jazzy-mujoco-ros2-control
```

**Humble:** no binary package is published; build the pinned tag from source:

```bash
cd ~/ws/src
git clone -b 0.0.3 https://github.com/ros-controls/mujoco_ros2_control.git
rosdep install --from-paths mujoco_ros2_control -y --ignore-src
colcon build --packages-up-to mujoco_ros2_control
```

> **The version pin matters.** `r2d3_mujoco` targets `mujoco_ros2_control` **0.0.3**
> specifically. Newer releases move camera/lidar publishing out of the core
> hardware interface and into a separate `mujoco_ros2_control_plugins`
> package driven by its own plugins config, instead of reading the `<sensor>`
> blocks this package emits in `mujoco_inputs.urdf.xacro`. Building a newer
> tag against this package's URDF/config as-is will silently drop the
> camera and lidar.

---

## First Launch

The first time you launch (per robot model, per world), `r2d3_mujoco`
bootstraps a converter virtual environment and converts the xacro-generated
URDF into an MJCF scene. This is the slow path:

1. **venv bootstrap** (first time ever on a machine): `mujoco_ros2_control`
   creates and populates a Python venv at `~/.ros/ros2_control/.venv`. This
   can take a few minutes the very first time.
2. **URDF → MJCF conversion**: the converter does STL → OBJ conversion for
   every mesh in the robot description (32 meshes for this robot) and
   writes the result into a per-model cache directory,
   `~/.ros/r2d3_mujoco/<model>/` (e.g. `~/.ros/r2d3_mujoco/65b/`,
   `~/.ros/r2d3_mujoco/75b/`). On a machine with warm mesh/venv caches this
   takes roughly 20–30 seconds; budget several minutes for a genuinely cold
   cache. `ensure_mjcf.py` also patches the converter output to fix a lidar
   self-occlusion issue (see the Troubleshooting table) before caching it.

Every subsequent launch with the **same** xacro-generated URDF and world
file is a cache hit — `ensure_mjcf.py` logs `cache hit` and republishes the
already-patched MJCF in about a second, skipping mesh conversion entirely.

The cache is automatically invalidated (a fresh conversion is triggered)
whenever the URDF (robot model, xacro args) or the world file content
changes. To force a reconversion regardless of cache state — e.g. after
editing `mujoco_inputs.urdf.xacro` — pass `force_recompile:=true`, or
delete the cache directory outright:

```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py force_recompile:=true
# or
rm -rf ~/.ros/r2d3_mujoco
```

---

## 1. Simulation Only (MuJoCo + Controllers)

Starts the MuJoCo physics simulation, spawns the robot, and activates all
ros2_control controllers. No navigation, no MoveIt.

```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py
```

### Parameters

| Parameter         | Default              | Description                                     |
|--------------------|----------------------|--------------------------------------------------|
| `robot_model`       | `65b`                 | Robot arm variant: `65b` (6-DOF arms) or `75b` (7-DOF arms) |
| `world`              | `worlds/nav_empty.xml` | Full path to the MuJoCo scene XML             |
| `headless`           | `false`                | Run MuJoCo without the interactive Simulate window |
| `force_recompile`    | `false`                | Force URDF → MJCF recompilation even if cached |

### Example

```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py robot_model:=75b headless:=true
```

### What starts

| Node / Process                | Purpose                                          |
|--------------------------------|---------------------------------------------------|
| `ensure_mjcf.py`                | Cached URDF→MJCF conversion; publishes `/mujoco_robot_description` |
| `robot_state_publisher`         | Publishes `/robot_description` and `/tf`          |
| `mujoco_ros2_control_node`       | Physics simulation + `controller_manager`; publishes `/scan`, `/imu`, `/zed/zed_node/*`, `/ground_truth_odom` |
| `joint_state_broadcaster`        | Publishes `/joint_states`                          |
| `diff_drive_controller`          | AGV base velocity control                          |
| `left_arm_controller`            | Left arm joint trajectory control                  |
| `right_arm_controller`           | Right arm joint trajectory control                 |
| `platform_controller`            | Torso lift (gripper-style) control                 |
| `imu_sensor_broadcaster`         | Publishes `/imu`                                   |
| `stereo_concat`                  | Sim-only ZED shim: side-by-side stereo + rgb alias |
| `zed_points_container`           | Composable container converting depth + RGB into `/zed/zed_node/point_cloud/cloud_registered` |

### Teleop (manual driving)

Once the simulation is running, in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel \
  -p stamped:=true -p frame_id:=base_footprint
```

---

## 2. Test Nodes (AGV + Arm motion verification)

Simple scripted motions to verify controllers are working.
Requires the simulation from Step 1 to be already running.

### AGV motion test

Runs a sequence: forward → stop → rotate CW → stop → backward → stop.

```bash
ros2 run r2d3_test_nodes test_agv_motion --ros-args -p use_sim_time:=true
```

### Arm motion test

Sends joint trajectory goals: left arm wave → home → right arm wave → home.

```bash
ros2 run r2d3_test_nodes test_arm_motion --ros-args -p use_sim_time:=true
```

---

## 3. Navigation / Full Stack – MuJoCo + Nav2 + MoveIt2

Starts **everything**: MuJoCo simulation, Nav2 navigation (SLAM or
localization), and — unless disabled — MoveIt2 arm planning with a combined
RViz view. Self-contained: it reuses `dual_rm_navigation`'s SLAM/Nav2
sub-launches and `r2d3_bringup`'s MoveIt/RViz configuration, but does not
depend on Gazebo at all.

```bash
ros2 launch r2d3_mujoco bringup_sim.launch.py
```

### Parameters

| Parameter        | Default                            | Description                                      |
|-------------------|-------------------------------------|----------------------------------------------------|
| `robot_model`      | `65b`                                | Robot arm variant: `65b` or `75b`                  |
| `world`             | `r2d3_mujoco/worlds/nav_empty.xml`    | Full path to the MuJoCo scene XML                  |
| `mode`              | `slam`                               | `slam` for mapping, `localization` for saved map   |
| `slam_type`         | `slam_toolbox`                       | SLAM backend: `slam_toolbox`, `rtabmap`, or `rtabmap_depth_only` |
| `map`               | *(empty)*                             | Path to map YAML (required for localization with `slam_toolbox`) |
| `use_rviz`          | `true`                                | Launch combined Nav2 + MoveIt RViz                 |
| `use_moveit`        | `true`                                | Launch MoveIt2 `move_group` for arm planning       |
| `headless`          | `false`                               | Run MuJoCo without the Simulate window             |

### Examples

```bash
# SLAM Toolbox (default) – 2D LiDAR-based mapping, full stack
ros2 launch r2d3_mujoco bringup_sim.launch.py

# RTAB-Map with RGB-D + LiDAR fusion
ros2 launch r2d3_mujoco bringup_sim.launch.py slam_type:=rtabmap

# 75b arm variant, headless (no RViz, no Simulate window — useful for CI)
ros2 launch r2d3_mujoco bringup_sim.launch.py robot_model:=75b use_rviz:=false headless:=true

# Navigation only (no arm planning)
ros2 launch r2d3_mujoco bringup_sim.launch.py use_moveit:=false

# Localization mode – navigate a previously saved map
ros2 launch r2d3_mujoco bringup_sim.launch.py mode:=localization map:=/path/to/map.yaml
```

### Save a map

While in SLAM mode, once you've explored the environment:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Send a navigation goal

In RViz, use the **Nav2 Goal** tool (green arrow in the toolbar) to click a
destination on the map. For MoveIt arm planning, select a planning group
(`left_arm`, `right_arm`, or `platform`) in the **MotionPlanning** panel,
drag the interactive markers, and click **Plan** then **Execute**.

---

## 4. Extra MuJoCo Goodies

Running under `mujoco_ros2_control` gives you a few things Gazebo doesn't
expose directly:

| Feature                 | How                                                                 |
|--------------------------|----------------------------------------------------------------------|
| Ground-truth base odometry | `/ground_truth_odom` — published straight from the converter-added free joint (`floating_base_joint`), independent of the diff-drive controller's estimated odometry |
| Pause / resume            | `ros2 service call /mujoco_ros2_control_node/set_pause mujoco_ros2_control_msgs/srv/SetPause "{paused: true}"` (use `paused: false` to resume) |
| Single-step (requires pause first) | `ros2 service call /mujoco_ros2_control_node/step_simulation mujoco_ros2_control_msgs/srv/StepSimulation "{steps: 10}"` |
| Reset the world           | `ros2 service call /mujoco_ros2_control_node/reset_world mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: ''}"` |
| Interactive Simulate window | Omit `headless:=true` (or set it `false`, the default) to get the MuJoCo Simulate GUI. **Space** pauses/resumes; **→** single-steps while paused. |

---

## Package Architecture

```
r2d3_mujoco             MuJoCo sim: xacros, cached URDF->MJCF conversion, controllers, world, bringup
dual_rm_navigation       Nav2 stack, SLAM (slam_toolbox / RTAB-Map), localization, nav params (reused)
r2d3_bringup             MoveIt2 launch + combined RViz config (reused)
r2d3_test_nodes          Simple test executables for AGV and arm motion
```

| What you want to do                          | Launch command                                                              |
|------------------------------------------------|--------------------------------------------------------------------------------|
| MuJoCo + controllers only                       | `ros2 launch r2d3_mujoco mujoco_sim.launch.py`                                |
| MuJoCo + Nav2 (SLAM Toolbox) + MoveIt2 (full stack) | `ros2 launch r2d3_mujoco bringup_sim.launch.py`                          |
| MuJoCo + Nav2 (RTAB-Map + LiDAR)                | `ros2 launch r2d3_mujoco bringup_sim.launch.py slam_type:=rtabmap`            |
| MuJoCo + Nav2 (RTAB-Map depth only)             | `ros2 launch r2d3_mujoco bringup_sim.launch.py slam_type:=rtabmap_depth_only` |
| Test AGV motion                                 | `ros2 run r2d3_test_nodes test_agv_motion`                                     |
| Test arm motion                                 | `ros2 run r2d3_test_nodes test_arm_motion`                                     |
| Teleop (keyboard)                               | See teleop command in Section 1                                                |

---

## Aiming the wrist cameras

Each wrist carries a RealSense D435 publishing under `/left_wrist/**` and
`/right_wrist/**` (`color/image_raw`, `color/camera_info`,
`depth/image_rect_raw`, `depth/color/points`).

Aim is set in one place, read by both sims:

```
ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml
```

Per arm variant (`65b` / `75b`) and side:

- `tilt` — radians about the mount Y. **Negative tilts the camera down**
  toward the gripper. This is usually the only value you need.
- `pan` — radians about the mount Z (left/right sweep).
- `xyz` / `rpy` — the physical housing pose. These describe the bracket in the
  wrist mesh; leave them alone unless the hardware changes.

`pan: 0, tilt: 0` is the camera looking straight down the tool axis (the
wrist's +Z, the direction the gripper reaches along) — so it sees whatever the
gripper is pointed at, and an arm pointing down sees the floor. Roughly
`tilt: -0.28` centres the gripper tip in frame.

Rebuild after editing — this workspace does not use `--symlink-install`, so
xacro reads the installed copy:

```bash
colcon build --packages-select dual_rm_description && source install/setup.bash
```

> **MuJoCo-specific:** the wrist `depth/color/points` clouds run through
> `depth_image_proc` with `approximate_sync` enabled. MuJoCo does not stamp
> colour and depth frames bit-identically, so the default exact-time
> synchronizer starves — it dropped roughly 85–90% of frames before
> `approximate_sync` was turned on. The trade-off: colour and depth can be
> paired from slightly different instants, so a fast-moving arm may show
> some edge misalignment ("smearing") in the cloud. That is an accepted
> trade-off of the sync policy, not a bug.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| venv bootstrap slow, hangs, or fails | Corrupted/incomplete `mujoco_ros2_control` converter venv | Delete `~/.ros/ros2_control/.venv` and relaunch — it rebuilds from scratch |
| Launch fails at conversion with a `patch count` ERROR | The converter's generated MJCF no longer matches `ensure_mjcf.py`'s lidar-visibility patterns (upstream `mujoco_ros2_control` version change, renamed chassis meshes, resized lidar housing, etc.) — this is a **hard failure by design**: publishing an unpatched model would silently resurrect the all-`-1.0` `/scan` bug | Inspect the cached `mujoco_description_formatted.xml` and update the matching patterns in `patch_lidar_housing_visibility()` (`r2d3_mujoco/scripts/ensure_mjcf.py`) |
| Stale / wrong-looking simulation after editing xacros | Cache hit is reusing an old MJCF because the checksum still matches (e.g. you edited a file the checksum doesn't cover) | Delete `~/.ros/r2d3_mujoco/` or relaunch with `force_recompile:=true` |
| `/zed/zed_node/point_cloud/cloud_registered` never publishes, but everything else works | `depth_image_proc` isn't installed — `zed_points_container` fails to load the `PointCloudXyzrgbNode` component (`Could not find requested resource in ament index` in the log), but this failure doesn't affect any other node | `sudo apt install ros-$ROS_DISTRO-depth-image-proc`, then relaunch |
| `/joint_states` / `/imu` publish slower than expected (e.g. ~55 Hz instead of the configured 100 Hz) | The machine can't keep the MuJoCo physics step running at real-time; sensor publish rates drop proportionally to the achieved sim rate | Not a config error — expected on slower/loaded machines. Reduce other load, or treat published rates as approximate |
| EGL / GLFW errors when the camera renders (e.g. `libEGL warning`, `Failed to create OpenGL context`) | Headless GPU / software-rendering driver quirks | Usually harmless — camera rendering still works via software fallback; if it doesn't, try `headless:=true` to skip the interactive Simulate window (camera rendering is independent of it) |
| `~/robot_description` service errors on Humble | Humble's `controller_manager` expects a `~/robot_description` topic remap that Jazzy doesn't need | Already handled automatically — `mujoco_sim.launch.py` adds the `~/robot_description -> /robot_description` remap only when `ROS_DISTRO=humble` |
| `ros2 topic hz` on a camera topic reports an implausibly high or noisy rate | Stale simulator/bridge processes from an earlier session survived a naive `pkill` and are duplicate-publishing onto the same topic name; `ros2 topic hz` silently aggregates across all publishers | Check publisher count first with `ros2 topic info -v <topic>`; if more than one, find and kill the stragglers with `pgrep -af 'mujoco\|gz sim\|ros_gz_bridge\|ruby\|component_container'` |
| Wheels slip / robot drifts off a straight line | Contact friction / solver tuning | Tunable in `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`: the shared `collision` default class's `friction`/`condim`, and `<option noslip_iterations="...">` |
| Underpan / chassis pan looks invisible in RViz's camera image or in the Simulate window | **Intentional.** The lidar mounts flush against the chassis pan (`base_link_underpan`, `body_base_link`); `ensure_mjcf.py` zeroes their rgba alpha so the lidar's own rays don't self-occlude. Alpha is a rendering property too, so this also hides them from the RGB/depth camera. The camera looks outward/forward and doesn't normally frame the underpan, so this is accepted as a cosmetic trade-off, not a bug | None needed; if camera-visible chassis geometry is ever required, it needs per-consumer geom duplication upstream (out of scope for this package) |
| `right_arm_controller` / `left_arm_controller` stay `inactive` on the **75b** (7-DOF) variant, with `Unable to activate controller ... command interface 'l_joint7/position'/'r_joint7/position' is not available` | `l_joint7`/`r_joint7` are present in the generated URDF's `<ros2_control>` block and in the converted MJCF's actuator list, but `mujoco_ros2_control` 0.0.3's URDF parsing does not export a `command_interface` for either — reproduced consistently on both cache-miss and cache-hit launches. `joint_state_broadcaster`, `diff_drive_controller`, `platform_controller`, and `imu_sensor_broadcaster` are unaffected | Known upstream limitation of `mujoco_ros2_control` 0.0.3 with a 7-joint-per-arm `<ros2_control>` block; not fixable from `r2d3_mujoco/` files. The 65b (6-DOF) variant is unaffected — use it if full arm control is required today |
