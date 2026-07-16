# Gazebo camera bore fix via sim-only mount frame

**Date:** 2026-07-16
**Issue:** [#11](https://github.com/Open-Droids-robot/R2D3_ros2/issues/11) — Gazebo camera image points 90° left
**Status:** Approved

## Problem

The sim overlay yaws the whole mechanical tree +90° about Z at
`base_footprint_to_base` (mesh → Nav2 reconciliation), so `camera_link` +X =
`base_footprint` +Y. A Gz `rgbd_camera` renders along its **mounting frame's
+X** — `gz_frame_id` only *labels* the published output, it does not reorient
the render. Result: `/camera/image` looks 90° to the robot's left, and since
commit `c7bbfec` (MuJoCo fix) the render also disagrees with its own
`camera_optical_frame` label (nav-forward), so consumers place the cloud wrong.

MuJoCo was fixed by reorienting the frame its camera hangs on
(`camera_optical_joint` rpy `-π/2 0 -π`). That does nothing for Gazebo, whose
bore follows the mounting link, not the optical frame.

## Investigation findings (why the first attempt was abandoned)

The first fix (branch `stash/gz-camera-bore-fix`) put
`<pose>0 0 0 0 0 -π/2</pose>` inside the Gz `<sensor>` block. Its geometry was
correct (sdformat lumping composes the pose: final SDF sensor pose
`… 0 0 -1.5708` on `head_link2`), but the operator's bringup broke during
verification and the change was reverted with a "do not rotate the sensor
pose" constraint (issue #11 postmortem comment). Log forensics
(`~/.ros/log/create_268766_…`, Jul 15 21:14) show that failing run's spawner
auto-discovered a stale foreign Gz world named `w` and waited on
`/world/w/create` for 3.4 h — the known stale-gz-server trap. Regardless, per
operator direction the sensor-`<pose>` approach is **not** to be retried; this
design achieves the same SDF a different way and must be verified against the
full bringup on a clean machine.

## Fix

In `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`
(shared sim sensor macro):

1. Add a sim-only, massless mount frame, child of `camera_link`, yawed −90°:

   ```xml
   <link name="camera_gz_frame"/>
   <joint name="camera_gz_joint" type="fixed">
       <parent link="camera_link"/>
       <child link="camera_gz_frame"/>
       <origin xyz="0 0 0" rpy="0 0 ${-pi/2}"/>
   </joint>
   ```

2. Re-reference the existing Gz sensor block to it:
   `<gazebo reference="camera_gz_frame">` (was `camera_link`).

Nothing else changes. No sensor `<pose>`, no mesh geometry, no mechanical
joint angles; core description `camera_joint` stays `rpy="0 0 0"`
(real-faithful); MuJoCo fix `c7bbfec` untouched.

### Why this works

sdformat's URDF fixed-joint reduction lumps `camera_gz_frame` into
`head_link2` and migrates the attached sensor with the composed transform.
Statically verified (2026-07-16): flattened full-robot URDF → `gz sdf -p` is
valid; sensor lands on `head_link2` with pose `-0.0032 -0.0519 0.0616 0 0
-1.5708` → render bore = nav +X, upright, consistent with the
`camera_optical_frame` label. This is the exact analog of the MuJoCo fix:
reorient the frame the camera hangs on.

### MuJoCo impact

The macro is shared (`r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro` includes it).
MuJoCo gains one massless fixed link — structurally identical to the existing
`camera_optical_frame` link it already digests — and ignores `<gazebo>`
blocks. Verified as part of the test suite (below).

## Testing

- **Regression test:** adapt `test_gz_camera_bore.py` from
  `stash/gz-camera-bore-fix` (chain-math already written) into
  `ros2_rm_robot/dual_rm_simulation/test/`. Assertions:
  1. Gz render bore (mount-frame +X at q=0) = `base_footprint` +X, upright.
  2. Gz-derived optical axes match the URDF `camera_optical_frame` label.
  Compute the mount orientation from the joint chain to `camera_gz_frame`
  (no sensor `<pose>` parsing).
- **MuJoCo guard:** existing `r2d3_mujoco/test/test_camera_optical_frame.py`
  must still pass; MuJoCo model must still compile.

## Verification protocol (mandatory, from the postmortem)

1. `pgrep -af "gz sim"` **empty** before any run — stale servers survive naive
   pkill and the spawner auto-discovers their worlds (the exact failure that
   sank attempt #1).
2. `colcon build` the touched packages first — install space is copy-not-
   symlink; unbuilt runs silently test the old URDF.
3. Full `ros2 launch r2d3_bringup bringup_sim.launch.py`: robot spawns exactly
   once, `/camera/image` in `rqt_image_view` looks forward (drive direction),
   Nav2/SLAM unaffected.
4. Static checks (xacro flatten, `gz sdf --check`) are necessary but **not
   sufficient** — the live bringup eyeball is the acceptance gate.

## Out of scope

- Spawner hardening (`-world nav_world` on `ros_gz_sim create`) — declined for
  now; revisit separately if stale-world spawn hangs recur.
- Any change to `base_footprint_to_base` yaw or the URDF-unification
  architecture.
