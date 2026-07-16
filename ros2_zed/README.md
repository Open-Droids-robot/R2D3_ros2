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
  with `publish_tf:=false publish_urdf:=false` — robot_state_publisher owns
  the ZED TF from the URDF; zed_node must not double-publish it.
