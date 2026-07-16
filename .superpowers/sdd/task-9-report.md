# Task 9: Full verification sweep — Report

Date: 2026-07-16
Branch: sim
Workspace: ~/code/r2d3 (ROS 2 Jazzy)
Env: `source /opt/ros/jazzy/setup.bash && source ~/code/r2d3/install/setup.bash`

## Step 1: Clean full build — PASS

Command:
```bash
cd ~/code/r2d3 && colcon build
```

Output (tail):
```
Starting >>> dual_rm_description
Starting >>> servo_interfaces
...
Starting >>> zed_msgs
Finished <<< zed_msgs [0.91s]
...
Summary: 29 packages finished [2.12s]
```

- No `colcon build` errors.
- `zed_wrapper` / `zed_components` did NOT appear in the build output (COLCON_IGNOREd) — expected.
- `zed_msgs` built successfully — expected.

**PASS**

## Step 2: All unit/static tests — PASS

Command:
```bash
cd ~/code/r2d3
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v
```

Output (summary):
```
collected 55 items

src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py::TestZedOpticalFrames::test_bores_point_nav_forward_both_eyes PASSED
src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py::TestZedOpticalFrames::test_stereo_baseline PASSED
src/R2D3_ros2/r2d3_mujoco/test/test_ensure_mjcf.py (28 tests) PASSED
src/R2D3_ros2/r2d3_mujoco/test/test_wait_for_sim_ready.py (7 tests) PASSED
src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py (4 tests) PASSED
src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py (5 tests) PASSED
src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_world_materials.py::TestWorldMaterials::test_visual_materials_define_diffuse PASSED

============================== 55 passed in 0.68s ==============================
```

All 55 tests PASSED, none skipped.

**PASS**

## Step 3: Flatten every URDF entrypoint — PASS

Command:
```bash
cd ~/code/r2d3
source /opt/ros/jazzy/setup.bash && source install/setup.bash
for m in 65b 75b; do
  xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=$m > /dev/null && echo "gz $m OK"
  xacro src/R2D3_ros2/r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro arm_model:=$m > /dev/null && echo "mj $m OK"
done
```

Output:
```
gz 65b OK
mj 65b OK
gz 75b OK
mj 75b OK
```

All four entrypoints flatten cleanly for both arm models.

**PASS**

## Step 4 (best-effort): Live MuJoCo smoke test — PASS

Launched headless (default 65b arm model, per known-issue note that
mujoco_ros2_control 0.0.3 drops the 7th joint interface for 75b — irrelevant
to camera topics):

```bash
cd ~/code/r2d3
source /opt/ros/jazzy/setup.bash && source install/setup.bash
nohup ros2 launch r2d3_mujoco mujoco_sim.launch.py headless:=true > mujoco_launch.log 2>&1 &
# waited ~135s total for MJCF conversion + controller activation before probing topics
```

Launch log tail showed all controllers (right_arm_controller, diff_drive_controller,
left_arm_controller) loaded, configured, and activated successfully — sim fully up.

### Topic list

```
$ ros2 topic list | grep zed
/zed/zed_node/depth/depth_registered
/zed/zed_node/left/camera_info
/zed/zed_node/left/image_rect_color
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/rgb/camera_info
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/right/camera_info
/zed/zed_node/right/depth_unused
/zed/zed_node/right/image_rect_color
/zed/zed_node/stereo/image_rect_color
```

All nine §1 contract topics present, including `rgb/image_rect_color` +
`rgb/camera_info` (stereo_concat) and `point_cloud/cloud_registered`
(depth_image_proc — the previously-noted apt gap did NOT reproduce this run;
depth_image_proc/zed_points_container started fine, no ENV-GAP needed here).

### Hz measurement — left image

```
$ ros2 topic hz /zed/zed_node/left/image_rect_color --window 10   (12s sample)
average rate: 15.058
average rate: 13.629
average rate: 14.917
average rate: 14.932
average rate: 15.010
average rate: 14.958
average rate: 14.800
average rate: 14.990
average rate: 14.876
average rate: 15.158
```
~15 Hz confirmed (matches expected "~15 Hz").

### Stereo width

```
$ ros2 topic echo /zed/zed_node/stereo/image_rect_color --once --field width
2560
```
Matches expected `2560`.

### Left frame_id

```
$ ros2 topic echo /zed/zed_node/left/image_rect_color --once --field header.frame_id
zed_left_camera_frame_optical
```
Matches expected `zed_left_camera_frame_optical`.

### Additional checks (brief's extra ask)

```
$ ros2 topic echo /zed/zed_node/rgb/camera_info --once --field width
1280

$ ros2 topic echo /zed/zed_node/point_cloud/cloud_registered --once --field header.frame_id
zed_left_camera_frame_optical
```
Both present and correctly framed.

**PASS** — all assertions met, no environment gaps encountered this run.

### Cleanup

```bash
pkill -f mujoco
pkill -f "ros2 launch r2d3_mujoco"
pgrep -af "mujoco|ros2 launch|ros2_control_node"   # empty (only the pgrep invocation itself matched)
```
Confirmed no stale sim processes remain.

## Step 5

Belongs to the controller (not this task) per brief instructions — plan
checkbox updates / commit / finishing-a-development-branch handoff left to
the calling session.

## Summary

| Step | Result |
|---|---|
| 1. Clean full build | PASS |
| 2. Unit/static tests (55) | PASS |
| 3. Xacro flatten (4 entrypoints) | PASS |
| 4. Live MuJoCo smoke test | PASS |

No defects found. No source changes made. No commits created.

## Final-review fix round

Fixing the two Critical + several Important/minor findings from the final
review of the ZED 2 head camera branch. Human decision: adopt the vendored
wrapper's v5.x topic names everywhere (the earlier work used v4 names; the
vendored `ros2_zed/zed-ros2-wrapper` is v5.x/master and renamed them).

### Step 0 — authoritative v5 names, derived from vendored source

Read `ros2_zed/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_video_depth.cpp`:

- `initVideoDepthPublishers()` builds image topics via a `make_topic(sensor,
  color_mode, rect_raw, type)` lambda: `topic = mTopicRoot + sensor +
  color_mode + rect_raw + type`, then `resolve_topic_name(topic)`.
  `mTopicRoot = "~/"` (declared in `zed_camera_component.hpp:418`), which
  resolves relative to the node namespace/name (`/zed/zed_node/`).
  - `mLeftTopic = make_topic("left/", "color/", "rect/", "image")`
  - `mRightTopic = make_topic("right/", "color/", "rect/", "image")`
  - `mRgbTopic = make_topic("rgb/", "color/", "rect/", "image")`
  - `mStereoTopic = make_topic("stereo/", "color/", "rect/", "image")`
  - Resolved: `/zed/zed_node/left/color/rect/image`,
    `/zed/zed_node/right/color/rect/image`,
    `/zed/zed_node/rgb/color/rect/image`,
    `/zed/zed_node/stereo/color/rect/image` — matches the prompt's expected
    names exactly.
- `mDepthTopic = mTopicRoot + "depth/depth_registered"` →
  `/zed/zed_node/depth/depth_registered` (unchanged from v4, confirmed).
- `mPointcloudTopic = mTopicRoot + "point_cloud/cloud_registered"` →
  `/zed/zed_node/point_cloud/cloud_registered` (unchanged, confirmed).
- `make_cam_info_pub(topic)` calls `image_transport::getCameraInfoTopic(topic)`.
  Checked `/opt/ros/jazzy/include/image_transport/image_transport/camera_common.hpp`:
  its doc comment says it "form[s] the camera info topic name, **sibling** to
  the base topic" — i.e. it replaces the last path segment (`image`) with
  `camera_info`, it does NOT append `/camera_info` as a child of the full
  image topic. So camera_info topics are:
  - `/zed/zed_node/left/color/rect/camera_info`
  - `/zed/zed_node/right/color/rect/camera_info`
  - `/zed/zed_node/rgb/color/rect/camera_info`
  - `/zed/zed_node/stereo/color/rect/camera_info` (unused in this repo — no
    consumer subscribes to stereo camera_info)

**Verified v5 topic list actually used in this repo:**
```
/zed/zed_node/left/color/rect/image
/zed/zed_node/left/color/rect/camera_info
/zed/zed_node/right/color/rect/image
/zed/zed_node/right/color/rect/camera_info
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
/zed/zed_node/stereo/color/rect/image
/zed/zed_node/depth/depth_registered            (unchanged)
/zed/zed_node/point_cloud/cloud_registered      (unchanged)
```

### A. Topic rename sweep (Critical 1) — DONE

Renamed the old v4 names (`left/image_rect_color`, `left/camera_info`, etc.)
to the v5 names above in:

1. `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py` — bridge
   remappings' ROS-side names (Gz-side `/zed/left/...` sensor topics
   untouched, as instructed).
2. `ros2_rm_robot/dual_rm_simulation/scripts/stereo_concat.py` — all 6
   pub/sub topic strings + docstring.
3. `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro` — both
   `zed_left`/`zed_right` sensor blocks' `info_topic`/`image_topic`;
   `depth_topic` for `zed_left` left unchanged (v5-unchanged name); added a
   comment on `zed_right`'s `depth_topic` noting `right/depth_unused` is
   intentionally out-of-contract junk (never consumed).
4. `r2d3_mujoco/launch/mujoco_sim.launch.py` — `depth_image_proc` remaps
   (`rgb/camera_info`, `rgb/image_rect_color` targets renamed; the
   left-hand/plugin-internal names like `"rgb/image_rect_color"` as remap
   *keys* are untouched — those are `depth_image_proc`'s own fixed internal
   subscription names, not part of our ZED contract).
5. `ros2_rm_robot/dual_rm_navigation/launch/rtabmap.launch.py` and
   `rtabmap_depth_only.launch.py` — the two LEFT-eye remap lines
   (`rgb/image`, `rgb/camera_info`); comment already accurately said "LEFT
   eye", left as-is.
6. `r2d3_mujoco/scripts/wait_for_sim_ready.py` — `--camera-topic` default
   renamed; module docstring's "Ready means all of" list was missing the
   camera-image bullet entirely (only listed 3 of 4 signals used by
   `signals_ready()`/argparse description) — added the missing bullet.
7. `r2d3_mujoco/test/test_wait_for_sim_ready.py` — all `camera_topic=` /
   expected-missing-list strings renamed.
8. `simulation_quickstart_gz.md` — the one `/zed/zed_node/left/...`
   troubleshooting-table mention renamed.
   `simulation_quickstart_mujoco.md` — checked; its `/zed/zed_node/*` and
   `/zed/zed_node/point_cloud/cloud_registered` mentions were already
   contract-name-agnostic or already-correct (point cloud name is unchanged
   v4→v5), no edit needed there.
   `simulation_quickstart.md` — checked; contains no `zed`/`zed_node`
   mentions at all, no edit needed.
9. `docs/superpowers/specs/2026-07-16-zed2-head-camera-design.md` — updated
   the §1 contract table (all four image+camera_info pairs) and the §4
   rtabmap remap code snippet to v5 names; added a note under the table
   explaining the v5.1 upstream rename is why the doc's original names
   changed.

### B. Wrapper params opt-in (Critical 2) — DONE

Checked `ros2_zed/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml`:
`publish_left_right: false` and `publish_stereo: false` live directly under
the `video:` parameter group (confirmed via the file's top-level key list —
`video` is the section header both lines fall under, at file lines 54 and
57). `publish_rgb` (line 53) is already `true` by default, so RGB alone
needed no override.

Added to `ros2_r2d3_apps/r2d3_bringup/config/zed2_params.yaml` under a new
`video:` block:
```yaml
video:
  publish_left_right: true
  publish_stereo: true
```

### C. publish_map_tf explicit (ledger 7) — DONE

Confirmed `publish_map_tf` is a real launch arg in
`ros2_zed/zed-ros2-wrapper/zed_wrapper/launch/zed_camera.launch.py` (declared
line 199, wired to `pos_tracking.publish_map_tf` at line 442, declared as a
`DeclareLaunchArgument` at line 558). Added
`'publish_map_tf': 'false'` to `launch_arguments` in
`ros2_r2d3_apps/r2d3_bringup/launch/zed2.launch.py`, alongside a docstring
note explaining why (nav owns map->odom).

### D. Known sim/real deltas doc (Important 3+4, minor imu note) — DONE

Added a "Known sim/real deltas" section to `ros2_zed/README.md` covering:
1. Point-cloud `frame_id`: real wrapper stamps `zed_left_camera_frame`
   (X-forward), sims stamp `zed_left_camera_frame_optical`; both frames
   exist in TF (fixed transform from the zed2 macro), so TF-based consumers
   are unaffected — only frame_id string comparisons would break.
2. Image encoding: real wrapper publishes `bgra8` (or `bgr8` with
   `enable_24bit_output`), sims publish `rgb8`; `stereo_concat.py`'s
   `hconcat_images` already branches on `.encoding`, so it's fine as-is —
   documented as a trap for any future pixel-touching consumer.
3. `sensors.publish_imu_tf` must stay `true` on the robot — confirmed via
   `grep -rn "zed_imu_link" dual_rm_description/.../zed2.urdf.xacro`
   returned nothing: the URDF macro has no IMU link, so the wrapper's own
   static IMU TF broadcast is load-bearing on hardware.

Also fixed `zed2.launch.py`'s docstring: removed the "byte-for-byte" claim
(false — see the deltas above) and pointed readers at the new README
section instead.

### E. Dead include (minor 5) — DONE

`r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro`: confirmed via
`grep -n "zed2_camera\|zed2_sim\|xacro:zed"` that
`zed2_sim.urdf.xacro` was `<xacro:include>`d but its macro was never
instantiated anywhere in the file (or transitively — MuJoCo gets its ZED
frames from `dual_rm_description`'s `body_head_platform` via the core
description include). Removed the dead include and rewrote the adjacent
comment to explain where the ZED frames actually come from and that the
Gz-sensor macro is never instantiated in MuJoCo.

(Caught during verification: my first attempt at this comment used a
literal `--` inside an XML comment, which XML forbids — `xacro` failed to
parse with "not well-formed (invalid token)". Fixed by rewording without
`--`; re-verified with a clean xacro flatten below.)

### Verification

**1. Build** — `cd ~/code/r2d3 && colcon build --packages-select
dual_rm_simulation r2d3_mujoco dual_rm_navigation r2d3_bringup`:
```
Starting >>> dual_rm_simulation
Finished <<< dual_rm_simulation [0.05s]
Starting >>> dual_rm_navigation
Finished <<< dual_rm_navigation [0.04s]
Starting >>> r2d3_bringup
Finished <<< r2d3_bringup [0.04s]
Starting >>> r2d3_mujoco
Finished <<< r2d3_mujoco [0.05s]
Summary: 4 packages finished [0.34s]
```
PASS.

**2. Unit tests** —
`python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v`:
```
55 passed in 0.65s
```
(2 GPU/render-dependent bore tests that skipped on the first pre-fix run
passed cleanly on the post-fix rerun — environmental, not related to these
changes; all topic/name-relevant tests — `test_stereo_concat.py`,
`test_wait_for_sim_ready.py`, both bore tests — passed both times.) PASS.

**3. Xacro flatten** — both entrypoints:
```
xacro r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro        -> exit 0
xacro ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro -> exit 0
```
(65b default arm model; the mujoco entrypoint initially failed with an XML
parse error caused by the `--` typo above — fixed and reverified, exit 0
both.) PASS.

**4. Old-name sweep** —
```
grep -rn "image_rect_color" ros2_rm_robot r2d3_mujoco ros2_r2d3_apps simulation_quickstart*.md \
  --include="*.py" --include="*.xacro" --include="*.md" \
  | grep -v ros2_zed | grep -v docs/ | grep -v .superpowers
```
One hit:
```
r2d3_mujoco/launch/mujoco_sim.launch.py:141:  ("rgb/image_rect_color", "/zed/zed_node/left/color/rect/image"),
```
This is fine: it's the LEFT side of a `depth_image_proc` remap tuple — the
`PointCloudXyzrgbNode` component's own fixed internal subscription name
(`rgb/image_rect_color`), which is *remapped from*, not one of our
published ZED contract topics. The right-hand side (our topic) is already
the new v5 name. No other hits. PASS.

**5. Live MuJoCo smoke test with new names** —

Hit a real environment trap during this step (not a code bug, documented
for future reference / added to memory): two earlier stray launches from
this same session were left running because `pkill -9 -f "a\|b\|c"`
silently does NOT do alternation — `pkill -f` uses POSIX extended regex,
where `\|` is a literal escaped pipe character, not alternation (that's
`grep`/BRE syntax). The "kill" appeared to succeed only because the
matching `pgrep` check used the same broken pattern and also found nothing.
This produced a genuinely confusing first result: `ros2 topic list | grep
zed` showed BOTH old and new topic names simultaneously (an old stale
`stereo_concat.py`/`mujoco_ros2_control_node` from 15:46 was still
publishing old names alongside a freshly-rebuilt instance publishing new
names). Fixed by using unescaped `|` in the `pkill -f`/`pgrep -f` patterns,
confirmed zero stray processes via `ps aux`, then relaunched once cleanly.

Clean run, single instance confirmed via `ps aux` (one
`mujoco_ros2_control_node`, one `stereo_concat.py`, one
`zed_points_container` component_container), sim-ready log line seen:

```
$ ros2 topic list | grep zed
/zed/zed_node/depth/depth_registered
/zed/zed_node/left/color/rect/camera_info
/zed/zed_node/left/color/rect/image
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/rgb/color/rect/camera_info
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/right/color/rect/camera_info
/zed/zed_node/right/color/rect/image
/zed/zed_node/right/depth_unused
/zed/zed_node/stereo/color/rect/image
```
All v5-contract topics present, zero old names.

```
$ ros2 topic echo /zed/zed_node/stereo/color/rect/image --once --field width
2560
```
Expected 2560 (2x 1280 HD720 width). PASS.

```
$ ros2 topic echo /zed/zed_node/left/color/rect/image --once --field header.frame_id
zed_left_camera_frame_optical
```
Expected `zed_left_camera_frame_optical`. PASS.

**Cleanup**: `pkill -9 -f "ensure_mjcf|mujoco_ros2_control_node|component_container|stereo_concat|ros2_control_node|ros2 launch r2d3_mujoco|robot_state_publisher.*launch_params|controller_manager|spawner"`,
then verified with `pgrep -af ...` (no matches, exit 1) and a final `ps aux`
grep (0 matching processes). No stale sim processes remain.

### Final-review fix round summary

| Fix | Result |
|---|---|
| Step 0: derive v5 names from vendored source | DONE — verified against prompt's expected names, exact match |
| A. Topic rename sweep (9 files) | DONE |
| B. Wrapper params opt-in (zed2_params.yaml) | DONE |
| C. publish_map_tf explicit (zed2.launch.py) | DONE |
| D. Known sim/real deltas doc (ros2_zed/README.md) | DONE |
| E. Dead include removed (r2d3_mujoco.urdf.xacro) | DONE |
| Build | PASS |
| Unit tests | 55 passed |
| Xacro flatten (both entrypoints) | PASS |
| Old-name sweep | 1 expected/benign hit, no real leftovers |
| Live MuJoCo smoke test (new names) | PASS |
