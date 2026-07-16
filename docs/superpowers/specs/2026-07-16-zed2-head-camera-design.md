# ZED 2 Head Camera — Design

**Date:** 2026-07-16
**Status:** Approved design, pending implementation plan

## Goal

Replace the head camera (generic `camera_link` + RGB-D sim sensor, RealSense on
the real robot) with a **ZED 2 stereo camera**, ZED-native end-to-end: the
simulation (Gazebo Harmonic + MuJoCo) publishes the same topics and TF frames
that `zed-ros2-wrapper` publishes on the real robot, so sim↔real is a
launch-file choice with zero adapter code.

**Out of scope:** wrist cameras (future RealSense integration — the vendored
`ros2_realsense2/` tree stays untouched), Isaac Sim / ZED SDK simulation mode,
ZED positional tracking / object detection in sim.

## Decisions made during brainstorming

| Question | Decision |
|---|---|
| Sim approach | ZED-faithful Gz/MuJoCo sim; `zed_ros2_wrapper` runs only on real hardware (it requires ZED SDK + CUDA; its sim mode only supports Isaac Sim) |
| Stereo scope in sim | True left/right stereo pair + depth + side-by-side stereo topic |
| Wrapper delivery | Vendored into the repo (like `ros2_realsense2/`), COLCON_IGNOREd by default |
| RealSense cleanup | None — kept for future wrist cameras |
| Head mount | `camera_link` (mesh + joint) fully replaced by the ZED 2 model |
| Naming | zed-ros2-wrapper defaults, `camera_name:=zed` → `/zed/zed_node/...` |
| RTAB-Map input | Explicitly the **left** eye stream (never the `rgb/` alias, never the side-by-side topic) |

## §1 — Interface contract

Topics published **in sim** (● = sim publishes; the real wrapper publishes these
plus its full native set — IMU, pose, disparity, etc. — for free):

| Topic | Sim | Notes |
|---|---|---|
| `/zed/zed_node/left/image_rect_color` + `left/camera_info` | ● | Left eye = RGB reference eye |
| `/zed/zed_node/right/image_rect_color` + `right/camera_info` | ● | True stereo pair |
| `/zed/zed_node/rgb/image_rect_color` + `rgb/camera_info` | ● | Alias of left |
| `/zed/zed_node/stereo/image_rect_color` | ● | Side-by-side rectified L+R, double-width image. Sim: `stereo_concat` node. Real: published natively by zed_node |
| `/zed/zed_node/depth/depth_registered` | ● | Depth registered to the **left** eye (matches real ZED behavior) |
| `/zed/zed_node/point_cloud/cloud_registered` | ● | Point cloud |

Frames (from `zed_macro.urdf.xacro`):
`zed_camera_link → zed_camera_center → zed_{left,right}_camera_frame →
zed_{left,right}_camera_optical_frame`, with the ZED 2's 120 mm baseline baked
in. `zed_left_camera_optical_frame` is the frame_id of RGB, depth, and point
cloud — the canonical head-camera optical frame. The old `camera_link` /
`camera_optical_frame` / `camera_gz_frame` names disappear entirely; no aliases.

## §2 — Robot description (`dual_rm_description`)

- **Copy** `zed_macro.urdf.xacro` + the ZED 2 mesh from zed-ros2-wrapper into
  `dual_rm_description` (pattern: `realsense2_description` vs
  `realsense2_camera`). The robot model never `$(find)`s SDK-gated packages, so
  every dev machine builds and visualizes without CUDA.
- **`body_head_platform.urdf.xacro`**: delete the `camera_link` link and
  `camera_joint`; instantiate `<xacro:zed_camera name="zed" model="zed2"
  parent="head_link2">` at the old camera origin
  (`xyz="-0.0032391 -0.051866 0.061606"`). This origin is a starting point;
  the real mount offset must be calibrated on hardware (tracked as a follow-up,
  not part of this change).
- **Single mount-yaw compensation point**: the ZED mounts via an intermediate
  `zed_mount_joint` whose yaw is a xacro argument. The core description passes
  its real-faithful value; the Gz/MuJoCo sim overlay passes the mesh→nav de-yaw
  (today's `-pi/2` at `base_footprint_to_base`). Everything below the mount —
  left/right frames, optical frames, mesh — is then correct once, in both
  worlds. This **retires** the per-frame hacks in `depth_camera.urdf.xacro`
  (`camera_gz_frame`, the extra `-pi/2` in the optical joint; see issue #11
  and commit 9a01958). Exact angles are pinned by the updated bore tests, not
  eyeballed. Per the issue #11 postmortem: compensation is frame-level only —
  never a `<pose>` inside `<sensor>`.

## §3 — Simulation

### Gazebo Harmonic (`dual_rm_simulation`)

- `urdf/sensors/depth_camera.urdf.xacro` is retired, replaced by
  `urdf/sensors/zed2_sim.urdf.xacro`:
  - **Left eye**: `rgbd_camera` sensor on `zed_left_camera_frame` (X-forward,
    nav-correct via §2 mount-yaw) → RGB + depth + points,
    `gz_frame_id = zed_left_camera_optical_frame`.
  - **Right eye**: plain `camera` sensor on `zed_right_camera_frame` → RGB only,
    `gz_frame_id = zed_right_camera_optical_frame`.
  - ZED 2 optics: `horizontal_fov ≈ 1.919` (110°), 1280×720 default
    (configurable), clip 0.3–20 m, gaussian noise as today.
- `gz_sim.launch.py` bridge: the four `/camera/*` entries are replaced by the
  §1 topic set, remapped from Gz sensor topics to `/zed/zed_node/...` names.
  The `rgb/*` alias is produced by bridging the left topics twice or a relay —
  whichever is fewer moving parts, decided at plan time.
- **`stereo_concat`** (new node in `dual_rm_simulation`): `message_filters`
  exact-time sync on left + right (sim stamps are identical), `hconcat`,
  publish `/zed/zed_node/stereo/image_rect_color` with the left header.
  Launched only in sim (the real wrapper publishes this topic natively).

### MuJoCo (`r2d3_mujoco`)

- Two `<camera>` entries in `mujoco_inputs.urdf.xacro` at the
  `zed_left_camera_optical_frame` / `zed_right_camera_optical_frame` sites;
  fovy recomputed for the ZED 2 vertical FOV (~70° at 16:9). Depth from the
  left camera. Same ROS topic names; `stereo_concat` reused.
- **Parity requirement**: both sims publish the identical §1 topic set.

## §4 — Real robot, vendoring, consumers

### Vendoring

- Clone `zed-ros2-wrapper` (+ `zed_msgs`) into `ros2_zed/`, mirroring the
  `ros2_realsense2/` layout.
- `ros2_zed/zed-ros2-wrapper/COLCON_IGNORE` is **committed** — dev machines
  build clean by default. On the robot, remove the ignore file (one-liner
  documented in `ros2_zed/README.md`). `zed_msgs` (pure interfaces) builds
  everywhere and is not ignored.

### Real bringup (`ros2_r2d3_apps/r2d3_bringup`)

- `zed2.launch.py` + `zed2_params.yaml`: include the wrapper launch with
  `camera_name:=zed`, `camera_model:=zed2`, and **`publish_tf:=false`** —
  robot_state_publisher owns the ZED frames via the URDF; the wrapper must not
  double-publish TF.

### Consumers

- `rtabmap.launch.py` remaps (pinned to the **left** eye; RTAB-Map must never
  receive the double-width `stereo/*` image):

  ```python
  ('rgb/image',       '/zed/zed_node/left/image_rect_color'),
  ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
  ('depth/image',     '/zed/zed_node/depth/depth_registered'),
  ```

  Left is also the physically correct choice: depth is registered to the left
  eye, so image, camera_info, and depth share
  `zed_left_camera_optical_frame`.
- `rtabmap_params.yaml`, rviz configs, and any other `/camera/*` or
  `camera_optical_frame` references updated to the new names in the same
  change.

## Testing

- `dual_rm_simulation/test/test_gz_camera_bore.py`: updated — left camera bores
  nav-forward under the new mount-yaw scheme; **new assertion**: right camera
  sits exactly 120 mm along the left camera's baseline axis.
- `r2d3_mujoco/test/test_camera_optical_frame.py`: same treatment for both
  optical frames.
- New unit test for `stereo_concat`: two synthetic images in → double-width
  image out, header taken from left.

## Error handling

- `stereo_concat` uses exact-time sync; if either eye stalls it publishes
  nothing rather than stale pairs.
- `wait_for_sim_ready.py` gains one ZED topic in its readiness checklist so
  missing bridge entries fail loudly at startup.

## Risks / notes

- The mount-yaw refactor touches the carefully-tuned issue #11 frame
  compensation; the updated bore tests are the safety net and must pass in
  both sims before this lands.
- Vendored wrapper is buildable only with ZED SDK + CUDA; the committed
  COLCON_IGNORE keeps dev machines green.
- Real mount origin on `head_link2` is inherited from the old camera and needs
  hardware calibration later.
