# ros2_zed — vendored ZED ROS 2 stack

Vendored copies of Stereolabs' ROS 2 packages (Apache-2.0), mirroring the
`ros2_realsense2/` pattern. Pinned upstream SHAs are recorded in the
`.zed-ros2-*.sha` files.

| Directory | Package(s) | Builds on |
|---|---|---|
| `zed-ros2-interfaces/` | `zed_msgs` (interfaces + meshes) | every machine |
| `zed-ros2-wrapper/` | `zed_wrapper`, `zed_components`, ... | **robot only** (needs ZED SDK ≥ 5.2 + CUDA) — `COLCON_IGNORE`d by default |

## Enable on the robot

```bash
rm ros2_zed/zed-ros2-wrapper/COLCON_IGNORE
cd ~/code/r2d3 && colcon build --packages-up-to zed_wrapper
```

Do NOT commit the COLCON_IGNORE removal — dev machines without the SDK must
keep building clean.

## Notes

- The robot description does NOT depend on these packages: the ZED 2 model
  is a self-contained copy in `dual_rm_description` (urdf/sensors/zed2.urdf.xacro).
- The real camera is launched via `r2d3_bringup/launch/zed2.launch.py`
  with `publish_tf:=false publish_urdf:=false publish_map_tf:=false` —
  robot_state_publisher owns the ZED TF from the URDF; zed_node must not
  double-publish it, and nav owns map->odom.

## Known sim/real deltas

The sim (Gz + MuJoCo) and the real wrapper both publish the same v5.x
topic contract (see
`docs/superpowers/specs/2026-07-16-zed2-head-camera-design.md` §1), but a
few implementation details differ and consumers must tolerate both:

- **Point-cloud `frame_id`**: the real wrapper stamps `zed_left_camera_frame`
  (X-forward, ROS optical-free convention) on
  `/zed/zed_node/point_cloud/cloud_registered`; the sims stamp
  `zed_left_camera_frame_optical` instead. Both frames exist in the TF tree
  (the zed2 macro publishes the fixed transform between them), so any
  consumer that does its own TF lookup works either way — just don't assume
  the frame_id string itself matches sim vs. real.
- **Image encoding**: the real wrapper publishes `bgra8` (or `bgr8` with
  `video.enable_24bit_output:=true`); the sims publish `rgb8`. Consumers
  that touch raw pixel data (e.g. `stereo_concat`'s `hconcat_images`) branch
  on `encoding` already; don't assume one specific encoding.
- **`sensors.publish_imu_tf` stays `true`** on the robot (wrapper default) —
  the `zed2.urdf.xacro` macro deliberately omits a `zed_imu_link`, so the
  wrapper's own static IMU TF broadcast is what completes the TF tree on
  hardware. Don't disable it to "match" sim; sim has no IMU frame to omit in
  the first place.
