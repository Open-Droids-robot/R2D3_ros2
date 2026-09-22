# R2D3 Gazebo Harmonic Simulation – Quick Start Guide

> **Prefer not to install ROS 2 locally?** `./droid up` from the repository root
> runs this whole simulation in a container and serves the GUI to your browser,
> on Linux, macOS, Jetson or a headless cloud instance. See
> [docs/container.md](docs/container.md).

## Prerequisites

```bash
# Build the workspace
cd <your-workspace-root>          # e.g. ~/code/r2d3
colcon build
source install/setup.bash
```

> **Tip:** Always source `install/setup.bash` in every new terminal.

### Rebuild after editing — yes, every time

This workspace is built **without** `--symlink-install`, so `install/` holds
plain *copies*, not links. At runtime everything resolves through the install
space: `$(find pkg)` in a xacro, `xacro.load_yaml()` on a config, launch files,
world files, params. **Editing a file in `src/` has no effect until you
rebuild** — the sim keeps reading the previous copy.

```bash
colcon build --packages-select <pkg> && source install/setup.bash
```

This applies to plain data files too, which is the surprising part: change a
YAML, relaunch without rebuilding, and the old value is still in force with no
warning. It looks exactly like "the setting does nothing", so it tends to send
you debugging code that was never wrong. For description/config-only packages
the rebuild is ~1 s — it is only copying files.

If you would rather have live edits, `colcon build --symlink-install` links
non-compiled assets instead of copying them and removes this entirely. It is a
whole-workspace choice: mixing the two produces a confusing hybrid install
space, so switch with a clean `rm -rf build install` first, and expect every
contributor and doc to assume the same mode.

---

## 1. Simulation Only (Gazebo + Controllers)

Starts Gazebo Harmonic (headless rendering), spawns the robot, and activates
all ros2_control controllers. No navigation, no MoveIt.

```bash
ros2 launch dual_rm_simulation gz_sim.launch.py
```

### Parameters

| Parameter        | Default      | Description                              |
|------------------|--------------|------------------------------------------|
| `robot_model`    | `65b`        | Robot arm variant: `65b` or `75b`        |
| `world`          | `nav_empty.sdf` | Full path to Gz Sim world SDF file   |
| `gz_verbosity`   | `1`          | Gz Sim verbosity level (`0`–`4`)         |

### Example

```bash
ros2 launch dual_rm_simulation gz_sim.launch.py robot_model:=75b gz_verbosity:=2
```

### What starts

| Node / Process              | Purpose                                   |
|-----------------------------|-------------------------------------------|
| `gazebo` (Gz Sim)           | Physics simulation (headless rendering)   |
| `robot_state_publisher`     | Publishes `/robot_description` and `/tf`  |
| `ros_gz_bridge`             | Bridges `/clock`, `/scan`, `/imu`, `/zed/zed_node/*` |
| `joint_state_broadcaster`   | Publishes `/joint_states`                 |
| `diff_drive_controller`     | AGV base velocity control                 |
| `left_arm_controller`       | Left arm joint trajectory control         |
| `right_arm_controller`      | Right arm joint trajectory control        |
| `platform_controller`       | Torso lift (gripper-style) control        |

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

## 3. Navigation Only (Gazebo + Nav2)

Starts the full simulation **plus** a SLAM backend and the Nav2 navigation stack.
No MoveIt arm planning.

```bash
ros2 launch dual_rm_navigation bringup_sim.launch.py
```

### Parameters

| Parameter        | Default                            | Description                                      |
|------------------|------------------------------------|--------------------------------------------------|
| `robot_model`    | `65b`                              | Robot arm variant: `65b` or `75b`                |
| `world`          | `nav_empty.sdf`                    | Full path to Gz Sim world SDF file               |
| `mode`           | `slam`                             | `slam` for mapping, `localization` for saved map |
| `slam_type`      | `slam_toolbox`                     | SLAM backend (see table below)                   |
| `map`            | *(empty)*                          | Path to map YAML (required for localization with slam_toolbox) |
| `use_rviz`       | `true`                             | Launch RViz2 with Nav2 view                      |
| `nav2_params`    | `config/nav2_params.yaml`          | Full path to Nav2 parameter file                 |
| `slam_params`    | `config/slam_toolbox_params.yaml`  | Full path to SLAM Toolbox parameter file         |
| `rtabmap_params` | `config/rtabmap_params.yaml`       | Full path to RTAB-Map parameter file             |

### SLAM Backends

| `slam_type` value    | Sensors used         | Registration method | Best for                          |
|----------------------|----------------------|---------------------|-----------------------------------|
| `slam_toolbox`       | 2D LiDAR             | Scan matching       | Fast 2D mapping, lightweight      |
| `rtabmap`            | RGB-D + 2D LiDAR     | ICP (LiDAR-based)   | Rich mapping with loop closure    |
| `rtabmap_depth_only` | RGB-D only           | Visual features     | When no LiDAR is available        |

### Examples

```bash
# SLAM Toolbox (default) – 2D LiDAR-based mapping
ros2 launch dual_rm_navigation bringup_sim.launch.py

# RTAB-Map with RGB-D + LiDAR fusion
ros2 launch dual_rm_navigation bringup_sim.launch.py slam_type:=rtabmap

# RTAB-Map depth camera only (no LiDAR)
ros2 launch dual_rm_navigation bringup_sim.launch.py slam_type:=rtabmap_depth_only

# Localization mode (slam_toolbox) – navigate a previously saved map
ros2 launch dual_rm_navigation bringup_sim.launch.py \
  mode:=localization map:=/path/to/map.yaml

# Localization mode (RTAB-Map) – uses previously built RTAB-Map database
ros2 launch dual_rm_navigation bringup_sim.launch.py \
  mode:=localization slam_type:=rtabmap

# Headless (no RViz)
ros2 launch dual_rm_navigation bringup_sim.launch.py use_rviz:=false
```

### Save a map

While in SLAM mode, once you've explored the environment:

```bash
# Save 2D occupancy grid (works with any SLAM backend)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

> **Note:** RTAB-Map also saves its own database (`~/.ros/rtabmap.db`) automatically.
> This database is reused when launching in localization mode with `slam_type:=rtabmap`.

### Send a navigation goal

In RViz, use the **Nav2 Goal** tool (green arrow in toolbar) to click a
destination on the map.

---

## 4. Full Stack – Nav2 + MoveIt2 (Unified Bringup)

Starts **everything**: Gazebo simulation, Nav2 navigation, MoveIt2 arm planning,
and a combined RViz view with both Nav2 panels and MoveIt MotionPlanning plugin.

```bash
ros2 launch r2d3_bringup bringup_sim.launch.py
```

### Parameters

| Parameter      | Default         | Description                                      |
|----------------|-----------------|--------------------------------------------------|
| `robot_model`  | `65b`           | Robot arm variant: `65b` or `75b`                |
| `world`        | `nav_empty.sdf` | Full path to Gz Sim world SDF file               |
| `mode`         | `slam`          | `slam` for mapping, `localization` for saved map |
| `map`          | *(empty)*       | Path to map YAML (required when `mode:=localization`) |
| `use_rviz`     | `true`          | Launch combined Nav2 + MoveIt RViz               |
| `use_moveit`   | `true`          | Launch MoveIt2 move_group for arm planning       |

### Examples

```bash
# Full stack with defaults
ros2 launch r2d3_bringup bringup_sim.launch.py

# Navigation only (no arm planning)
ros2 launch r2d3_bringup bringup_sim.launch.py use_moveit:=false

# Headless (no RViz, useful for CI or remote testing)
ros2 launch r2d3_bringup bringup_sim.launch.py use_rviz:=false
```

### Using RViz with the full stack

The combined RViz window provides:

- **Nav2 Goal tool** – click on the map to send the AGV to a location
- **2D Pose Estimate tool** – set initial pose for localization mode
- **MotionPlanning panel** – plan and execute arm motions
  - Select a planning group (`left_arm`, `right_arm`, or `platform`)
  - Drag the interactive markers to set a target pose
  - Click **Plan** then **Execute** (or **Plan & Execute**)
- **Map / Costmap displays** – live map, local and global costmaps
- **Path displays** – green = global plan, blue = local plan
- **LaserScan** – red dots showing LiDAR readings
- **Camera** – RGB and depth images from the head-mounted depth camera

---

## Package Architecture

```
dual_rm_simulation      Gazebo sim, URDF, controllers, sensor bridge (LiDAR + IMU + depth camera)
dual_rm_navigation      Nav2 stack, SLAM (slam_toolbox / RTAB-Map), localization, nav params
r2d3_bringup            Unified bringup (Nav2 + MoveIt2 + combined RViz)
r2d3_test_nodes         Simple test executables for AGV and arm motion
```

| What you want to do                     | Launch command                                                            |
|-----------------------------------------|---------------------------------------------------------------------------|
| Gazebo + controllers only               | `ros2 launch dual_rm_simulation gz_sim.launch.py`                         |
| Gazebo + Nav2 (SLAM Toolbox)            | `ros2 launch dual_rm_navigation bringup_sim.launch.py`                    |
| Gazebo + Nav2 (RTAB-Map + LiDAR)        | `ros2 launch dual_rm_navigation bringup_sim.launch.py slam_type:=rtabmap` |
| Gazebo + Nav2 (RTAB-Map depth only)     | `ros2 launch dual_rm_navigation bringup_sim.launch.py slam_type:=rtabmap_depth_only` |
| Gazebo + Nav2 + MoveIt2 (full stack)    | `ros2 launch r2d3_bringup bringup_sim.launch.py`                         |
| Test AGV motion                         | `ros2 run r2d3_test_nodes test_agv_motion`                                |
| Test arm motion                         | `ros2 run r2d3_test_nodes test_arm_motion`                                |
| Teleop (keyboard)                       | See teleop command in Section 1                                           |

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

- `tilt` — **degrees** about the mount Y. **Negative tilts the camera down**
  toward the gripper. This is usually the only value you need.
- `pan` — **degrees** about the mount Z (left/right sweep).

All angles in that file are degrees; the xacro converts to radians at the one
point where it reads the file.
- `xyz` / `rpy` — the physical housing pose. These describe the bracket in the
  wrist mesh; leave them alone unless the hardware changes.

`pan: 0, tilt: 0` is the camera looking straight down the tool axis (the
wrist's +Z, the direction the gripper reaches along) — so it sees whatever the
gripper is pointed at, and an arm pointing down sees the floor. Roughly
`tilt: -16` centres the gripper tip in frame; `-45` angles it well across the
gripper.

> **An edit here does nothing until you rebuild.** This workspace does not use
> `--symlink-install`, so `$(find ...)` resolves to the *install* space and
> xacro keeps reading the previous copy of the file. If a tilt change appears
> to have no effect, this is almost certainly why — rebuild and relaunch.

Rebuild after editing — this workspace does not use `--symlink-install`, so
xacro reads the installed copy:

```bash
colcon build --packages-select dual_rm_description && source install/setup.bash
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Waiting messages on topic [robot_description]` | DDS discovery delay | Wait ~5s; the spawn uses `-string` to bypass this |
| `jump back in time` warnings | Non-monotonic `/clock` from multiple bridges | Ensure only one `ros_gz_bridge` is running |
| Map distorted on rotation | LiDAR self-detection | Already mitigated: `min_range: 0.55` in lidar + SLAM + costmaps |
| `libEGL warning: egl: failed to create dri2 screen` | GPU driver issue | Harmless; simulation runs with headless software rendering |
| Robot not moving in Gazebo but moves in RViz | Wheel collision geometry mismatch | Already fixed: cylinder primitives for drive wheels |
| Nav2 lifecycle manager fails to bring up nodes | Race condition at startup | Relaunch; transient issue with sim clock settling |
| RTAB-Map: `Waiting for data on topic ...` | Camera topics not bridged or not publishing | Verify `ros2 topic hz /zed/zed_node/left/color/rect/image` and `/zed/zed_node/depth/depth_registered` |
| RTAB-Map: no map generated | Textureless environment (depth_only mode) | Use `slam_type:=rtabmap` (adds LiDAR) or add visual features to the world |
| `ros2 topic hz` on a camera topic reports an implausibly high or noisy rate | Stale simulator/bridge processes from an earlier session survived a naive `pkill` and are duplicate-publishing onto the same topic name; `ros2 topic hz` silently aggregates across all publishers | Check publisher count first with `ros2 topic info -v <topic>`; if more than one, find and kill the stragglers with `pgrep -af 'mujoco\|gz sim\|ros_gz_bridge\|ruby\|component_container'` |
