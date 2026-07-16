# ZED 2 Head Camera Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the head camera with a ZED 2 stereo camera, ZED-native end-to-end: Gz Harmonic + MuJoCo sims publish the exact topics/frames `zed-ros2-wrapper` publishes on the real robot.

**Architecture:** A trimmed zed2 URDF macro in `dual_rm_description` (self-contained, no SDK deps) replaces `camera_link` on `head_link2` with one fixed physical `-pi/2` mount yaw that is correct in both the real-faithful core and the sim overlay. Both sims mount two `rgbd_camera`-equivalent sensors on the ZED left/right frames and rename topics to `/zed/zed_node/...`; a sim-only `stereo_concat` node synthesizes the side-by-side stereo topic and the rgb alias. The real wrapper is vendored under `ros2_zed/` behind a committed COLCON_IGNORE.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic (Gz Sim 8, `ros_gz_bridge`), mujoco_ros2_control, xacro, Python (rclpy, message_filters, numpy), RTAB-Map, zed-ros2-wrapper (vendored, ZED SDK 5.2 on robot only).

**Spec:** `docs/superpowers/specs/2026-07-16-zed2-head-camera-design.md`

## Global Constraints

- Workspace root is `~/code/r2d3` (this repo is `src/R2D3_ros2`). Build: `cd ~/code/r2d3 && colcon build --packages-select <pkg>` then `source install/setup.bash`.
- **Install-space copy trap:** xacro `$(find pkg)` resolves to the INSTALL space. After editing any `.xacro`, rebuild the owning package(s) BEFORE running any test or launch, or you test stale files.
- Frame names (wrapper ≥ v5): `zed_camera_link`, `zed_camera_center`, `zed_left_camera_frame`, `zed_left_camera_frame_optical`, `zed_right_camera_frame`, `zed_right_camera_frame_optical`. NEVER the old `*_optical_frame` order.
- Topic names: exactly `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/left/camera_info`, `/zed/zed_node/right/image_rect_color`, `/zed/zed_node/right/camera_info`, `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/rgb/camera_info`, `/zed/zed_node/stereo/image_rect_color`, `/zed/zed_node/depth/depth_registered`, `/zed/zed_node/point_cloud/cloud_registered`.
- ZED 2 constants: baseline `0.12` m, `bottom_slope` `0.05` rad, `optical_offset_x` `-0.01`, `height` `0.03`, hfov `1.9199` rad (110°), 1280×720, clip 0.3–20 m, 15 Hz.
- Sensor orientation is frame-level only — NEVER a `<pose>` inside a Gz `<sensor>` (issue #11 postmortem).
- Do not touch `ros2_realsense2/` or `ros2_total_demo/` (RealSense stays for future wrist cameras).
- `docs/` is gitignored in this repo; commit plan/spec docs with `git add -f`. Everything else adds normally.
- Every task leaves the workspace green: `r2d3_sim.urdf.xacro` and `r2d3_mujoco.urdf.xacro` must flatten cleanly at the end of every task.

---

### Task 1: ZED 2 URDF macro + mesh in dual_rm_description

Adds the ZED 2 model and mounts it on the head **alongside** the old
`camera_link` (which is removed later in Task 5), so nothing breaks.

**Files:**
- Create: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/zed2.urdf.xacro`
- Create: `ros2_rm_robot/dual_rm_description/dual_rm_description/meshes/common/zed2.stl` (downloaded)
- Modify: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro`

**Interfaces:**
- Produces: xacro macro `zed2_camera(name, parent, *origin)` creating links `${name}_camera_link`, `${name}_camera_center`, `${name}_left_camera_frame`, `${name}_left_camera_frame_optical`, `${name}_right_camera_frame`, `${name}_right_camera_frame_optical`; instantiated with `name="zed"` on `head_link2`. Later tasks mount Gz sensors on `zed_left_camera_frame`/`zed_right_camera_frame` and MuJoCo cameras on the `_frame_optical` sites.

- [x] **Step 1: Download the ZED 2 mesh**

```bash
curl -sL https://raw.githubusercontent.com/stereolabs/zed-ros2-interfaces/master/meshes/zed2.stl \
  -o ros2_rm_robot/dual_rm_description/dual_rm_description/meshes/common/zed2.stl
ls -la ros2_rm_robot/dual_rm_description/dual_rm_description/meshes/common/zed2.stl
```
Expected: file exists, size > 100 KB. If the URL 404s, find the mesh path with
`gh api repos/stereolabs/zed-ros2-interfaces/git/trees/master?recursive=1 --jq '.tree[].path' | grep -i stl`
and use that path.

- [x] **Step 2: Write the zed2 macro**

Create `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/zed2.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!--
  ZED 2 stereo camera model for the R2D3 head.

  Trimmed zed2-only copy of Stereolabs' zed_macro.urdf.xacro
  (zed-ros2-wrapper master, Apache-2.0, (c) 2025 Stereolabs), kept here so
  the robot description never $(find)s the SDK-gated vendored wrapper
  (ros2_zed/zed-ros2-wrapper is COLCON_IGNOREd on dev machines).

  Identical to upstream: link/joint names, 0.12 m baseline, -0.01 optical
  offset, 0.05 rad bottom_slope (the wedge-shaped case pitches the sensor
  block ~2.9 deg about Y at zed_camera_center), and the optical frame
  rotations. The real zed_node stamps zed_left_camera_frame_optical /
  zed_right_camera_frame_optical (wrapper >= v5 naming) on its output, so
  these frames are the sim/real contract.

  Changed from upstream: mesh path points into this package; added an
  inertial on zed_camera_center (ZED 2 weighs 166 g; upstream ships no
  inertial and the head dynamics want the mass); mag/baro/temp/GNSS links
  dropped (nothing consumes them).
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="zed2_camera" params="name:=zed parent *origin">
    <xacro:property name="baseline" value="0.12"/>
    <xacro:property name="height" value="0.03"/>
    <xacro:property name="bottom_slope" value="0.05"/>
    <xacro:property name="screw_offset_x" value="0.0"/>
    <xacro:property name="screw_offset_z" value="0.0"/>
    <xacro:property name="optical_offset_x" value="-0.01"/>

    <!-- Mounting point (the threaded screw hole in the bottom) -->
    <link name="${name}_camera_link"/>
    <joint name="${name}_mount_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${name}_camera_link"/>
      <xacro:insert_block name="origin"/>
    </joint>

    <!-- Camera center (the only link with geometry/mass) -->
    <link name="${name}_camera_center">
      <visual>
        <origin xyz="${screw_offset_x} 0 ${screw_offset_z}" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://$(find dual_rm_description)/meshes/common/zed2.stl"/>
        </geometry>
        <material name="zed2_mat">
          <color rgba="0.25 0.25 0.25 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="${screw_offset_x} 0 ${screw_offset_z}" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://$(find dual_rm_description)/meshes/common/zed2.stl"/>
        </geometry>
      </collision>
      <inertial>
        <!-- ZED 2: 166 g, box approx 0.033 x 0.175 x 0.030 m (X depth,
             Y baseline length, Z height) -->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.166"/>
        <inertia ixx="4.36e-04" ixy="0" ixz="0"
                 iyy="2.75e-05" iyz="0"
                 izz="4.39e-04"/>
      </inertial>
    </link>
    <joint name="${name}_camera_center_joint" type="fixed">
      <parent link="${name}_camera_link"/>
      <child link="${name}_camera_center"/>
      <origin xyz="0 0 ${height/2}" rpy="0 ${bottom_slope} 0"/>
    </joint>

    <!-- Left camera (RGB/depth reference eye) -->
    <link name="${name}_left_camera_frame"/>
    <joint name="${name}_left_camera_joint" type="fixed">
      <parent link="${name}_camera_center"/>
      <child link="${name}_left_camera_frame"/>
      <origin xyz="${optical_offset_x} ${baseline/2} 0" rpy="0 0 0"/>
    </joint>

    <link name="${name}_left_camera_frame_optical"/>
    <joint name="${name}_left_camera_joint_optical" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0.0 ${-pi/2}"/>
      <parent link="${name}_left_camera_frame"/>
      <child link="${name}_left_camera_frame_optical"/>
    </joint>

    <!-- Right camera -->
    <link name="${name}_right_camera_frame"/>
    <joint name="${name}_right_camera_joint" type="fixed">
      <parent link="${name}_camera_center"/>
      <child link="${name}_right_camera_frame"/>
      <origin xyz="${optical_offset_x} -${baseline/2} 0" rpy="0 0 0"/>
    </joint>

    <link name="${name}_right_camera_frame_optical"/>
    <joint name="${name}_right_camera_joint_optical" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0.0 ${-pi/2}"/>
      <parent link="${name}_right_camera_frame"/>
      <child link="${name}_right_camera_frame_optical"/>
    </joint>

  </xacro:macro>

</robot>
```

- [x] **Step 3: Instantiate on the head (old camera_link stays for now)**

In `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro`, add immediately after the opening `<robot ...>` / `mesh_path` property block (before the first `<link>`):

```xml
  <xacro:include filename="$(find dual_rm_description)/urdf/sensors/zed2.urdf.xacro"/>
```

and add at the end of the file, just before `</robot>` (after the `camera_joint` definition):

```xml
  <!-- ZED 2 head camera. The -pi/2 yaw is the physical mount: the ZED body
       is X-forward while the head meshes are modeled -Y-forward (the sim
       overlay's +pi/2 mesh->nav yaw at base_footprint_to_base makes -Y face
       nav +X). One fixed physical angle is therefore correct in BOTH the
       real-faithful core and the sim overlay - the two yaws cancel. No
       sim-only compensation frames needed (retires the issue #11
       camera_gz_frame mechanism). Origin inherited from the old camera_joint;
       real mount offset to be calibrated on hardware. Guarded by
       dual_rm_simulation/test/test_gz_camera_bore.py and
       r2d3_mujoco/test/test_camera_optical_frame.py. -->
  <xacro:zed2_camera name="zed" parent="head_link2">
    <origin xyz="-0.0032391 -0.051866 0.061606" rpy="0 0 ${-pi/2}"/>
  </xacro:zed2_camera>
```

- [x] **Step 4: Rebuild and verify the flattened URDF**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation r2d3_mujoco && source install/setup.bash
xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b > /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/07ffcd60-3562-48ee-8c7c-b5dd8fa7da8d/scratchpad/r2d3_sim_flat.urdf
grep -c 'zed_left_camera_frame_optical\|zed_right_camera_frame_optical\|zed_camera_center' /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/07ffcd60-3562-48ee-8c7c-b5dd8fa7da8d/scratchpad/r2d3_sim_flat.urdf
```
Expected: xacro exits 0; grep count ≥ 6 (each frame appears as link + joint child). Also flatten `r2d3_mujoco.urdf.xacro` the same way — exits 0.

- [x] **Step 5: Verify existing bore tests still pass (old camera untouched)**

```bash
cd ~/code/r2d3 && python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py -v
```
Expected: all PASS (not skipped — if skipped, the workspace isn't sourced/built).

- [x] **Step 6: Commit**

```bash
git add ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/zed2.urdf.xacro \
        ros2_rm_robot/dual_rm_description/dual_rm_description/meshes/common/zed2.stl \
        ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro
git commit -m "feat(description): add ZED 2 camera model on head (zed2_camera macro + mesh)"
```

---

### Task 2: stereo_concat sim node

Sim-only node that synthesizes the wrapper's side-by-side stereo topic and
the rgb alias from the simulated left/right streams.

**Files:**
- Create: `ros2_rm_robot/dual_rm_simulation/scripts/stereo_concat.py`
- Create: `ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py`
- Modify: `ros2_rm_robot/dual_rm_simulation/CMakeLists.txt`
- Modify: `ros2_rm_robot/dual_rm_simulation/package.xml`

**Interfaces:**
- Consumes: `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/right/image_rect_color`, `/zed/zed_node/left/camera_info` (published by Tasks 3/4).
- Produces: node executable `stereo_concat.py` (package `dual_rm_simulation`) publishing `/zed/zed_node/stereo/image_rect_color`, `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/rgb/camera_info`; pure function `hconcat_images(left: Image, right: Image) -> Image`.

- [x] **Step 1: Write the failing test**

Create `ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py`:

```python
"""Unit tests for the sim-only stereo_concat node's pure concat function."""
import sys
import unittest
from pathlib import Path

import numpy as np
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import stereo_concat  # noqa: E402


def _img(width, height, value, encoding="rgb8", frame="zed_left_camera_frame_optical"):
    bpp = {"rgb8": 3, "mono8": 1}[encoding]
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp.sec = 42
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * bpp
    msg.data = bytes([value]) * (height * msg.step)
    return msg


class TestHconcatImages(unittest.TestCase):
    def test_double_width_output(self):
        out = stereo_concat.hconcat_images(_img(4, 2, 10), _img(4, 2, 20))
        self.assertEqual(out.width, 8)
        self.assertEqual(out.height, 2)
        self.assertEqual(out.step, 8 * 3)
        self.assertEqual(len(out.data), 2 * 8 * 3)

    def test_row_layout_left_then_right(self):
        out = stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 1, 20))
        row = np.frombuffer(out.data, np.uint8)
        # left pixels (2 px * rgb) then right pixels, in the same row
        self.assertEqual(row[:6].tolist(), [10] * 6)
        self.assertEqual(row[6:].tolist(), [20] * 6)

    def test_header_taken_from_left(self):
        left = _img(2, 1, 10)
        right = _img(2, 1, 20, frame="zed_right_camera_frame_optical")
        out = stereo_concat.hconcat_images(left, right)
        self.assertEqual(out.header.frame_id, "zed_left_camera_frame_optical")
        self.assertEqual(out.header.stamp.sec, 42)
        self.assertEqual(out.encoding, "rgb8")

    def test_mismatched_height_raises(self):
        with self.assertRaises(ValueError):
            stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 2, 20))

    def test_mismatched_encoding_raises(self):
        with self.assertRaises(ValueError):
            stereo_concat.hconcat_images(_img(2, 1, 10), _img(2, 1, 20, encoding="mono8"))


if __name__ == "__main__":
    unittest.main()
```

- [x] **Step 2: Run test to verify it fails**

```bash
cd ~/code/r2d3 && python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py -v
```
Expected: FAIL/ERROR with `ModuleNotFoundError: No module named 'stereo_concat'`.

- [x] **Step 3: Write the node**

Create `ros2_rm_robot/dual_rm_simulation/scripts/stereo_concat.py` (mode 755):

```python
#!/usr/bin/env python3
"""Sim-only ZED topic shim: side-by-side stereo image + rgb alias.

The real zed-ros2-wrapper natively publishes
  /zed/zed_node/stereo/image_rect_color  (left|right side-by-side), and
  /zed/zed_node/rgb/image_rect_color(+camera_info)  (alias of the left eye).
Neither Gz Sim nor MuJoCo has a side-by-side stereo sensor, so this node
synthesizes both from the simulated left/right streams. It must NOT run on
the real robot -- zed_node already publishes these topics there.

Exact-time sync is deliberate: both sim eyes stamp identical sim time. If
either eye stalls, nothing is published (no stale pairs).
"""
import numpy as np
import rclpy
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

_BYTES_PER_PIXEL = {
    "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1,
}


def hconcat_images(left: Image, right: Image) -> Image:
    """Concatenate two Images horizontally (left | right). Header from left."""
    if left.height != right.height:
        raise ValueError(
            f"height mismatch: left={left.height} right={right.height}")
    if left.encoding != right.encoding:
        raise ValueError(
            f"encoding mismatch: left={left.encoding} right={right.encoding}")
    if left.encoding not in _BYTES_PER_PIXEL:
        raise ValueError(f"unsupported encoding: {left.encoding}")
    bpp = _BYTES_PER_PIXEL[left.encoding]

    l_rows = np.frombuffer(left.data, np.uint8).reshape(left.height, left.step)
    r_rows = np.frombuffer(right.data, np.uint8).reshape(right.height, right.step)
    # Drop any row padding beyond width*bpp before concatenating.
    l_rows = l_rows[:, : left.width * bpp]
    r_rows = r_rows[:, : right.width * bpp]

    out = Image()
    out.header = left.header
    out.height = left.height
    out.width = left.width + right.width
    out.encoding = left.encoding
    out.is_bigendian = left.is_bigendian
    out.step = out.width * bpp
    out.data = np.hstack((l_rows, r_rows)).tobytes()
    return out


class StereoConcat(Node):
    def __init__(self):
        super().__init__("stereo_concat")
        self._stereo_pub = self.create_publisher(
            Image, "/zed/zed_node/stereo/image_rect_color",
            qos_profile_sensor_data)
        self._rgb_pub = self.create_publisher(
            Image, "/zed/zed_node/rgb/image_rect_color",
            qos_profile_sensor_data)
        self._rgb_info_pub = self.create_publisher(
            CameraInfo, "/zed/zed_node/rgb/camera_info",
            qos_profile_sensor_data)

        left_sub = Subscriber(
            self, Image, "/zed/zed_node/left/image_rect_color",
            qos_profile=qos_profile_sensor_data)
        right_sub = Subscriber(
            self, Image, "/zed/zed_node/right/image_rect_color",
            qos_profile=qos_profile_sensor_data)
        self._sync = TimeSynchronizer([left_sub, right_sub], 5)
        self._sync.registerCallback(self._on_pair)

        self._info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/left/camera_info",
            self._on_left_info, qos_profile_sensor_data)

    def _on_pair(self, left: Image, right: Image):
        try:
            self._stereo_pub.publish(hconcat_images(left, right))
        except ValueError as e:
            self.get_logger().warn(f"skipping stereo pair: {e}",
                                   throttle_duration_sec=5.0)
        self._rgb_pub.publish(left)

    def _on_left_info(self, info: CameraInfo):
        self._rgb_info_pub.publish(info)


def main():
    rclpy.init()
    node = StereoConcat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

```bash
chmod +x ros2_rm_robot/dual_rm_simulation/scripts/stereo_concat.py
```

- [x] **Step 4: Run tests to verify they pass**

```bash
cd ~/code/r2d3 && python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py -v
```
Expected: 5 PASS.

- [x] **Step 5: Install the script and declare deps**

In `ros2_rm_robot/dual_rm_simulation/CMakeLists.txt`, after the existing `install(DIRECTORY ...)` block add:

```cmake
install(PROGRAMS
  scripts/stereo_concat.py
  DESTINATION lib/${PROJECT_NAME}
)
```

In `ros2_rm_robot/dual_rm_simulation/package.xml`, add alongside the existing exec deps:

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>message_filters</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
```
(skip any line already present)

- [x] **Step 6: Build and verify the executable resolves**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_simulation && source install/setup.bash
ros2 pkg executables dual_rm_simulation 2>/dev/null; ls install/dual_rm_simulation/lib/dual_rm_simulation/
```
Expected: `stereo_concat.py` listed in `lib/dual_rm_simulation/`.

- [x] **Step 7: Commit**

```bash
git add ros2_rm_robot/dual_rm_simulation/scripts/stereo_concat.py \
        ros2_rm_robot/dual_rm_simulation/test/test_stereo_concat.py \
        ros2_rm_robot/dual_rm_simulation/CMakeLists.txt \
        ros2_rm_robot/dual_rm_simulation/package.xml
git commit -m "feat(sim): add stereo_concat node (side-by-side stereo + rgb alias)"
```

---

### Task 3: Gazebo switches to the ZED sensors

**Files:**
- Create: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/zed2_sim.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py`
- Rewrite: `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py`

**Interfaces:**
- Consumes: `zed_left_camera_frame` / `zed_right_camera_frame` / `*_frame_optical` links (Task 1); `stereo_concat.py` executable (Task 2).
- Produces: xacro macro `zed2_sim_sensors()` (no params); Gz sensors named `zed_left`/`zed_right` on topics `zed/left`, `zed/right`; bridged+remapped ROS topics per the §1 contract.

- [x] **Step 1: Rewrite the bore test (failing first)**

Replace the entire content of `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py` with:

```python
"""Regression guard: both simulated ZED eyes must render along the robot's
nav-forward direction, upright, agree with the optical frames they label
their output with, and sit 120 mm apart (the ZED 2 baseline).

A Gz camera renders along the +X of the frame its ``<sensor>`` is mounted on
(+Z up, +Y left -- the SDF camera body convention). ``gz_frame_id`` only
LABELS the published output; it does NOT reorient the render. The sim overlay
yaws the whole mechanical tree +90deg about Z at ``base_footprint_to_base``
(mesh -> Nav2); the ZED's physical mount yaw (-90deg at zed_mount_joint,
the ZED body is X-forward while the head is mesh -Y-forward) cancels it, so
the sensors mount directly on the ZED left/right camera frames with NO
sim-only compensation frames (retires the issue #11 camera_gz_frame
mechanism). Per the issue #11 postmortem the Gz ``<sensor>`` ``<pose>`` must
NOT be used for orientation.

The zed2 case is wedge-shaped: zed_camera_center carries a +0.05 rad pitch
(bottom_slope, from Stereolabs' own model), so the expected bore is nav +X
pitched down by exactly that angle -- the assertions account for it. Any YAW
error (the issue #11 failure mode) still fails loudly.

NOTE: the xacro include resolves via $(find ...) -> the INSTALL space.
Rebuild (colcon build --packages-select dual_rm_description
dual_rm_simulation) after editing xacros, or this test sees old files.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"

SLOPE = 0.05      # zed2 bottom_slope (rad) -- keep equal to zed2.urdf.xacro
BASELINE = 0.12   # ZED 2 stereo baseline (m)


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF/SDF fixed-axis: Rz*Ry*Rx


def _flatten_urdf():
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise unittest.SkipTest("xacro not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [xacro_bin, str(XACRO), "arm_model:=65b"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise unittest.SkipTest(f"xacro failed (workspace not built?): {out.stderr[-400:]}")
    return out.stdout


def _joint_chain(urdf_str, child, root="base_footprint"):
    """Compose the fixed/neutral-config pose (R, p) from root down to `child`.

    Revolute/prismatic joints are evaluated at q=0, so only their <origin>
    contributes. Returns (R, p): child axes / position in root frame.
    """
    dom = minidom.parseString(urdf_str)
    joints = {}  # child_link -> (parent_link, R_origin, p_origin)
    for j in dom.getElementsByTagName("joint"):
        par = j.getElementsByTagName("parent")
        ch = j.getElementsByTagName("child")
        if not par or not ch:
            continue
        o = j.getElementsByTagName("origin")
        rpy, xyz = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if o:
            if o[0].getAttribute("rpy").strip():
                rpy = tuple(float(v) for v in o[0].getAttribute("rpy").split())
            if o[0].getAttribute("xyz").strip():
                xyz = tuple(float(v) for v in o[0].getAttribute("xyz").split())
        joints[ch[0].getAttribute("link")] = (
            par[0].getAttribute("link"), _rpy_to_R(*rpy), np.array(xyz))

    chain = []
    node = child
    while node != root:
        if node not in joints:
            raise AssertionError(f"no joint chain from {root} to {child} (stuck at {node})")
        parent, R, p = joints[node]
        chain.append((R, p))
        node = parent
    R_total, p_total = np.eye(3), np.zeros(3)
    for R, p in reversed(chain):  # root -> ... -> child
        p_total = p_total + R_total @ p
        R_total = R_total @ R
    return R_total, p_total


def _gz_camera_sensors(urdf_str):
    """Return {reference_link: (sensor_name, R_pose)} for all camera-type
    Gz sensors. R_pose is a direct-child <pose> rotation (identity if absent)
    so the bore assertions hold no matter which mechanism orients the render.
    """
    dom = minidom.parseString(urdf_str)
    found = {}
    for gz in dom.getElementsByTagName("gazebo"):
        for s in gz.getElementsByTagName("sensor"):
            if "camera" not in s.getAttribute("type"):
                continue
            rpy = (0.0, 0.0, 0.0)
            for pose in s.getElementsByTagName("pose"):
                if pose.parentNode is s:
                    vals = [float(v) for v in pose.firstChild.data.split()]
                    if len(vals) == 6:
                        rpy = tuple(vals[3:])
                    break
            found[gz.getAttribute("reference")] = (
                s.getAttribute("name"), _rpy_to_R(*rpy))
    if not found:
        raise AssertionError("no camera <sensor> found in any <gazebo> block")
    return found


# Gz builds the published optical frame from the sensor body frame by the fixed
# SDF-camera mapping: optical +Z = sensor +X (forward), optical +X = -sensor +Y
# (right), optical +Y = -sensor +Z (down). Columns are (optX, optY, optZ) in
# sensor-body coords.
_SENSOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])

# Expected sensor-body orientation in base_footprint: the +pi/2 overlay yaw and
# the -pi/2 physical mount yaw cancel, leaving only the zed2 bottom_slope
# pitch. bore = +X pitched down by SLOPE; up = +Z pitched forward by SLOPE.
_EXPECTED_BORE = np.array([math.cos(SLOPE), 0.0, -math.sin(SLOPE)])
_EXPECTED_UP = np.array([math.sin(SLOPE), 0.0, math.cos(SLOPE)])


class TestGzZedCameraBore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        cls.sensors = _gz_camera_sensors(cls.urdf)

    def test_sensors_mounted_on_zed_camera_frames(self):
        """Issue #11 postmortem: orientation lives in mount frames, never a
        sensor <pose>. Both eyes mount directly on the ZED camera frames."""
        self.assertEqual(
            set(self.sensors.keys()),
            {"zed_left_camera_frame", "zed_right_camera_frame"})

    def _sensor_R(self, ref):
        _, R_pose = self.sensors[ref]
        R_chain, _ = _joint_chain(self.urdf, ref)
        return R_chain @ R_pose

    def test_render_bores_nav_forward_both_eyes(self):
        for ref in ("zed_left_camera_frame", "zed_right_camera_frame"):
            R = self._sensor_R(ref)
            bore, up = R[:, 0], R[:, 2]
            np.testing.assert_allclose(
                bore, _EXPECTED_BORE, atol=1e-6,
                err_msg=f"{ref}: render bore should be nav +X (pitched down "
                        f"bottom_slope={SLOPE}), got {bore}")
            # Yaw error is the issue #11 failure mode -- assert it separately
            # and explicitly.
            self.assertLess(abs(bore[1]), 1e-6,
                            f"{ref}: render bore has a YAW error: {bore}")
            np.testing.assert_allclose(
                up, _EXPECTED_UP, atol=1e-6,
                err_msg=f"{ref}: image up should be nav +Z (pitched by "
                        f"bottom_slope={SLOPE}), got {up}")

    def test_render_matches_optical_frame_label(self):
        """The render axes must agree with the optical frame each sensor is
        LABELLED with (gz_frame_id), or consumers place clouds wrong."""
        for ref, side in (("zed_left_camera_frame", "left"),
                          ("zed_right_camera_frame", "right")):
            R_optical_render = self._sensor_R(ref) @ _SENSOR_TO_OPTICAL
            R_optical_label, _ = _joint_chain(
                self.urdf, f"zed_{side}_camera_frame_optical")
            np.testing.assert_allclose(
                R_optical_render, R_optical_label, atol=1e-6,
                err_msg=f"{side}: rendered optical axes disagree with the "
                        f"zed_{side}_camera_frame_optical label")

    def test_stereo_baseline(self):
        """Right eye sits exactly BASELINE along the left eye's -Y (the ZED
        left camera is the +Y eye of camera_center)."""
        R_left, p_left = _joint_chain(self.urdf, "zed_left_camera_frame")
        _, p_right = _joint_chain(self.urdf, "zed_right_camera_frame")
        offset_in_left = R_left.T @ (p_right - p_left)
        np.testing.assert_allclose(
            offset_in_left, [0.0, -BASELINE, 0.0], atol=1e-9,
            err_msg=f"stereo baseline wrong: {offset_in_left}")


if __name__ == "__main__":
    unittest.main()
```

- [x] **Step 2: Run to verify it fails for the right reason**

```bash
cd ~/code/r2d3 && python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```
Expected: FAIL — `test_sensors_mounted_on_zed_camera_frames` sees `{"camera_gz_frame"}` instead of the two ZED frames.

- [x] **Step 3: Write the ZED Gz sensor macro**

Create `ros2_rm_robot/dual_rm_simulation/urdf/sensors/zed2_sim.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!--
  ZED 2 stereo sensor pair for Gz Sim (Harmonic).

  Both eyes are rgbd_camera sensors mounted DIRECTLY on the ZED left/right
  camera frames (X-forward). No sim-only compensation frames: the physical
  -pi/2 mount yaw at zed_mount_joint (dual_rm_description zed2.urdf.xacro)
  cancels the sim overlay's +pi/2 mesh->nav yaw, so the frames already bore
  nav-forward. Per the issue #11 postmortem, never orient a sensor with a
  <pose> - only via the frame it is mounted on. Guarded by
  test/test_gz_camera_bore.py.

  The right eye is also an rgbd_camera purely for topic-layout consistency
  with the left (image/camera_info/depth_image/points under one prefix);
  only its image + camera_info get bridged in gz_sim.launch.py.

  ZED 2 optics: 110 deg hfov, 1280x720, 0.3-20 m.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:macro name="_zed2_gz_eye" params="side">
        <gazebo reference="zed_${side}_camera_frame">
            <sensor name="zed_${side}" type="rgbd_camera">
                <always_on>true</always_on>
                <update_rate>15</update_rate>
                <topic>zed/${side}</topic>
                <gz_frame_id>zed_${side}_camera_frame_optical</gz_frame_id>

                <camera name="zed_${side}">
                    <horizontal_fov>1.9199</horizontal_fov>  <!-- 110 deg, ZED 2 -->
                    <image>
                        <width>1280</width>
                        <height>720</height>
                        <format>R8G8B8</format>
                    </image>
                    <depth_camera>
                        <clip>
                            <near>0.3</near>
                            <far>20.0</far>
                        </clip>
                    </depth_camera>
                    <clip>
                        <near>0.3</near>
                        <far>20.0</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.007</stddev>
                    </noise>
                </camera>
            </sensor>
        </gazebo>
    </xacro:macro>

    <xacro:macro name="zed2_sim_sensors">
        <gazebo reference="zed_camera_center">
            <self_collide>false</self_collide>
        </gazebo>
        <xacro:_zed2_gz_eye side="left"/>
        <xacro:_zed2_gz_eye side="right"/>
    </xacro:macro>

</robot>
```

- [x] **Step 4: Switch r2d3_sim.urdf.xacro to the ZED sensors**

In `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro`:

Replace
```xml
    <xacro:include filename="$(find dual_rm_simulation)/urdf/sensors/depth_camera.urdf.xacro"/>
```
with
```xml
    <xacro:include filename="$(find dual_rm_simulation)/urdf/sensors/zed2_sim.urdf.xacro"/>
```
and replace
```xml
    <!-- Depth camera -->
    <xacro:depth_camera_sensor/>
```
with
```xml
    <!-- ZED 2 head camera (Gz stereo sensor pair) -->
    <xacro:zed2_sim_sensors/>
```

- [x] **Step 5: Update the bridge + launch stereo_concat**

In `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py`, replace the bridge `arguments` camera entries and add `remappings` so the node reads:

```python
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # ZED 2 head camera: two rgbd_camera sensors (topics zed/left,
            # zed/right). Right depth/points exist in Gz but are not bridged.
            '/zed/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/zed/left/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/zed/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            # Gz topic names -> the real zed-ros2-wrapper topic contract.
            ('/zed/left/image', '/zed/zed_node/left/image_rect_color'),
            ('/zed/left/camera_info', '/zed/zed_node/left/camera_info'),
            ('/zed/left/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/zed/left/points', '/zed/zed_node/point_cloud/cloud_registered'),
            ('/zed/right/image', '/zed/zed_node/right/image_rect_color'),
            ('/zed/right/camera_info', '/zed/zed_node/right/camera_info'),
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
```

After the `neck_servo_bridge` node definition, add:

```python
    # ── Sim-only ZED shim: side-by-side stereo + rgb alias ──────
    stereo_concat = Node(
        package='dual_rm_simulation',
        executable='stereo_concat.py',
        name='stereo_concat',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
```

and append `stereo_concat,` to the `LaunchDescription([...])` list (after `neck_servo_bridge`).

- [x] **Step 6: Rebuild, run the bore test**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```
Expected: 4 PASS. (MuJoCo's `test_camera_optical_frame.py` still passes too — old camera untouched until Task 5.)

- [x] **Step 7: Commit**

```bash
git add ros2_rm_robot/dual_rm_simulation/urdf/sensors/zed2_sim.urdf.xacro \
        ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro \
        ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py \
        ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py
git commit -m "feat(sim): Gz renders ZED 2 stereo pair on zed-ros2-wrapper topic contract"
```

---

### Task 4: MuJoCo switches to the ZED cameras

**Files:**
- Modify: `r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro`
- Modify: `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`
- Modify: `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro`
- Modify: `r2d3_mujoco/launch/mujoco_sim.launch.py`
- Rewrite: `r2d3_mujoco/test/test_camera_optical_frame.py`

**Interfaces:**
- Consumes: `zed_*` links/sites (Task 1); `stereo_concat.py` (Task 2).
- Produces: MJCF cameras `zed_left`/`zed_right`; ros2_control camera sensors publishing the §1 ZED topics; MuJoCo point cloud via `depth_image_proc` on `/zed/zed_node/point_cloud/cloud_registered`.

- [x] **Step 1: Rewrite the optical-frame test (failing first)**

Replace the entire content of `r2d3_mujoco/test/test_camera_optical_frame.py` with:

```python
"""Regression guard: both ZED optical frames must bore along the robot's
nav-forward direction, upright, and sit 120 mm apart (ZED 2 baseline).

The sim overlay yaws the whole mechanical tree +90deg at
base_footprint_to_base (mesh -> Nav2). The ZED's physical mount yaw (-90deg
at zed_mount_joint; the ZED body is X-forward, the head is mesh -Y-forward)
cancels it, so the standard optical rotations in the zed2 macro come out
globally correct with NO sim-only compensation (retires the extra -pi/2 the
old depth_camera.urdf.xacro carried; see commit 9a01958 / issue #11). The
zed2 case is wedge-shaped: zed_camera_center carries a +0.05 rad pitch
(bottom_slope, Stereolabs' own model), so the expected bore is nav +X
pitched down by exactly that angle. Any YAW error still fails loudly.

MuJoCo hangs its cameras on these frames' sites (mujoco_inputs.urdf.xacro),
so this guard covers the MuJoCo render orientation the same way
test_gz_camera_bore.py covers Gz.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_mujoco.urdf.xacro"

SLOPE = 0.05      # zed2 bottom_slope (rad) -- keep equal to zed2.urdf.xacro
BASELINE = 0.12   # ZED 2 stereo baseline (m)


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis: Rz*Ry*Rx


def _flatten_urdf():
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise unittest.SkipTest("xacro not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [xacro_bin, str(XACRO), "arm_model:=65b"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise unittest.SkipTest(f"xacro failed (workspace not built?): {out.stderr[-400:]}")
    return out.stdout


def _joint_chain(urdf_str, child, root="base_footprint"):
    """Compose the fixed/neutral-config pose (R, p) from root down to `child`.

    Revolute/prismatic joints are evaluated at q=0, so only their <origin>
    contributes. Returns (R, p): child axes / position in root frame.
    """
    dom = minidom.parseString(urdf_str)
    joints = {}  # child_link -> (parent_link, R_origin, p_origin)
    for j in dom.getElementsByTagName("joint"):
        par = j.getElementsByTagName("parent")
        ch = j.getElementsByTagName("child")
        if not par or not ch:
            continue
        o = j.getElementsByTagName("origin")
        rpy, xyz = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if o:
            if o[0].getAttribute("rpy").strip():
                rpy = tuple(float(v) for v in o[0].getAttribute("rpy").split())
            if o[0].getAttribute("xyz").strip():
                xyz = tuple(float(v) for v in o[0].getAttribute("xyz").split())
        joints[ch[0].getAttribute("link")] = (
            par[0].getAttribute("link"), _rpy_to_R(*rpy), np.array(xyz))

    chain = []
    node = child
    while node != root:
        if node not in joints:
            raise AssertionError(f"no joint chain from {root} to {child} (stuck at {node})")
        parent, R, p = joints[node]
        chain.append((R, p))
        node = parent
    R_total, p_total = np.eye(3), np.zeros(3)
    for R, p in reversed(chain):  # root -> ... -> child
        p_total = p_total + R_total @ p
        R_total = R_total @ R
    return R_total, p_total


# Expected optical axes in base_footprint: only the bottom_slope pitch
# survives (overlay +pi/2 yaw and mount -pi/2 yaw cancel).
_EXPECTED_BORE = np.array([math.cos(SLOPE), 0.0, -math.sin(SLOPE)])   # optical +Z
_EXPECTED_DOWN = np.array([-math.sin(SLOPE), 0.0, -math.cos(SLOPE)])  # optical +Y


class TestZedOpticalFrames(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()

    def test_bores_point_nav_forward_both_eyes(self):
        for side in ("left", "right"):
            R, _ = _joint_chain(self.urdf, f"zed_{side}_camera_frame_optical")
            bore = R[:, 2]   # optical +Z = viewing direction
            down = R[:, 1]   # optical +Y = down in image
            np.testing.assert_allclose(
                bore, _EXPECTED_BORE, atol=1e-6,
                err_msg=f"{side}: optical bore should be nav +X (pitched "
                        f"down bottom_slope={SLOPE}), got {bore}")
            self.assertLess(abs(bore[1]), 1e-6,
                            f"{side}: optical bore has a YAW error: {bore}")
            np.testing.assert_allclose(
                down, _EXPECTED_DOWN, atol=1e-6,
                err_msg=f"{side}: optical down should be nav -Z (pitched by "
                        f"bottom_slope={SLOPE}), got {down}")

    def test_stereo_baseline(self):
        R_left, p_left = _joint_chain(self.urdf, "zed_left_camera_frame_optical")
        _, p_right = _joint_chain(self.urdf, "zed_right_camera_frame_optical")
        # In the LEFT OPTICAL frame (X right, Y down, Z forward) the right
        # eye sits +BASELINE along +X.
        offset_in_left = R_left.T @ (p_right - p_left)
        np.testing.assert_allclose(
            offset_in_left, [BASELINE, 0.0, 0.0], atol=1e-9,
            err_msg=f"stereo baseline wrong: {offset_in_left}")


if __name__ == "__main__":
    unittest.main()
```

- [x] **Step 2: Run to verify it fails for the right reason**

```bash
cd ~/code/r2d3 && python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py -v
```
Expected: PASS actually — the zed frames already exist since Task 1 and the geometry is already correct. If it fails, STOP: the mount yaw or macro geometry is wrong — fix Task 1 before continuing. (The failing-first signal for this task is the MJCF camera rename below; this test locks the frames the cameras hang on.)

- [x] **Step 3: Swap the MuJoCo camera definitions**

In `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`, replace:

```xml
                <!-- RGB-D camera at the optical frame. fovy(deg) derived from the Gazebo
                     horizontal_fov of 1.047 rad at 640x480:
                     fovy = 2*atan(tan(1.047/2)*480/640) = 46.83 deg -->
                <camera site="camera_optical_frame" name="camera" fovy="46.83"
                        mode="fixed" resolution="640 480"/>
```

with:

```xml
                <!-- ZED 2 stereo pair at the optical-frame sites. fovy(deg)
                     derived from the Gazebo horizontal_fov of 1.9199 rad at
                     1280x720: fovy = 2*atan(tan(1.9199/2)*720/1280) = 77.56 deg.
                     Camera names must equal the ros2_control <sensor> names in
                     mujoco_ros2_control.urdf.xacro. -->
                <camera site="zed_left_camera_frame_optical" name="zed_left"
                        fovy="77.56" mode="fixed" resolution="1280 720"/>
                <camera site="zed_right_camera_frame_optical" name="zed_right"
                        fovy="77.56" mode="fixed" resolution="1280 720"/>
```

- [x] **Step 4: Swap the ros2_control camera sensors**

In `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro`, replace:

```xml
            <!-- RGB-D camera: sensor name must equal the MJCF camera name -->
            <sensor name="camera">
                <param name="frame_name">camera_optical_frame</param>
                <param name="info_topic">/camera/camera_info</param>
                <param name="image_topic">/camera/image</param>
                <param name="depth_topic">/camera/depth_image</param>
            </sensor>
```

with:

```xml
            <!-- ZED 2 stereo pair: sensor names must equal the MJCF camera
                 names (mujoco_inputs.urdf.xacro). Depth comes from the left
                 eye (the real ZED registers depth to the left eye). -->
            <sensor name="zed_left">
                <param name="frame_name">zed_left_camera_frame_optical</param>
                <param name="info_topic">/zed/zed_node/left/camera_info</param>
                <param name="image_topic">/zed/zed_node/left/image_rect_color</param>
                <param name="depth_topic">/zed/zed_node/depth/depth_registered</param>
            </sensor>
            <sensor name="zed_right">
                <param name="frame_name">zed_right_camera_frame_optical</param>
                <param name="info_topic">/zed/zed_node/right/camera_info</param>
                <param name="image_topic">/zed/zed_node/right/image_rect_color</param>
                <param name="depth_topic">/zed/zed_node/right/depth_unused</param>
            </sensor>
```

Then check whether `depth_topic` is optional in the installed interface:

```bash
grep -rn "depth_topic" $(ros2 pkg prefix mujoco_ros2_control 2>/dev/null)/.. 2>/dev/null | head -5 || \
grep -rn "depth_topic" ~/code/r2d3/src --include="*.cpp" --include="*.hpp" | grep -iv r2d3 | head -5
```
If the source shows `depth_topic` is optional (e.g. a `has_parameter`/default guard), delete the `depth_topic` line from the `zed_right` sensor. If mandatory (or you can't tell), keep `/zed/zed_node/right/depth_unused` — it's harmless and outside the wrapper's namespace contract.

- [x] **Step 5: Point the MuJoCo top-level xacro at the new sensor macro file**

In `r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro`, replace:

```xml
    <xacro:include filename="$(find dual_rm_simulation)/urdf/sensors/depth_camera.urdf.xacro"/>
```
with
```xml
    <xacro:include filename="$(find dual_rm_simulation)/urdf/sensors/zed2_sim.urdf.xacro"/>
```
and replace
```xml
    <xacro:depth_camera_sensor/>
```
with
```xml
    <!-- ZED 2 Gz sensor tags are ignored by the MuJoCo converter; the ZED
         links/frames themselves come from the core description (zed2 macro
         in body_head_platform). Nothing to instantiate here. -->
```
Also update the stale header comment on the includes (`camera_optical_frame` → `zed_*_camera_frame_optical` in the "only the links/joints ... matter" comment).

- [x] **Step 6: Update the MuJoCo launch (point cloud + stereo_concat)**

In `r2d3_mujoco/launch/mujoco_sim.launch.py`, replace the `depth_image_proc` remappings:

```python
                remappings=[
                    ("rgb/camera_info", "/zed/zed_node/left/camera_info"),
                    ("rgb/image_rect_color", "/zed/zed_node/left/image_rect_color"),
                    ("depth_registered/image_rect", "/zed/zed_node/depth/depth_registered"),
                    ("points", "/zed/zed_node/point_cloud/cloud_registered"),
                ],
```

and rename the container comment/name from `camera_points_container` / "/camera/points" to:

```python
    # -- /zed/zed_node/point_cloud/cloud_registered from left depth + info --
    pointcloud_container = ComposableNodeContainer(
        name="zed_points_container",
```

After the `neck_servo_bridge` node, add and register in the returned actions list:

```python
    # -- Sim-only ZED shim: side-by-side stereo + rgb alias --
    stereo_concat = Node(
        package="dual_rm_simulation",
        executable="stereo_concat.py",
        name="stereo_concat",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
```

- [x] **Step 7: Rebuild and run both sims' static tests**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation r2d3_mujoco && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v
```
Expected: all PASS (both bore tests, stereo_concat, wait_for_sim_ready).

- [x] **Step 8: Commit**

```bash
git add r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro \
        r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro \
        r2d3_mujoco/launch/mujoco_sim.launch.py r2d3_mujoco/test/test_camera_optical_frame.py
git commit -m "feat(mujoco): render ZED 2 stereo pair on zed-ros2-wrapper topic contract"
```

---

### Task 5: Remove the legacy camera

Now nothing references `camera_link`/`camera_optical_frame` — delete them.

**Files:**
- Modify: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro`
- Delete: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/gazebo/sim_gazebo.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_moveit_config/dual_rm_65b_moveit_config/config/dual_rm_65b_description.srdf`
- Modify: `ros2_rm_robot/dual_rm_moveit_config/dual_rm_75b_moveit_config/config/dual_rm_75b_description.srdf`
- Modify: `simulation_quickstart_gz.md`, `simulation_quickstart_mujoco.md`, `simulation_quickstart.md` (camera topic mentions)

**Interfaces:**
- Consumes: everything switched off the old camera (Tasks 3–4).
- Produces: a tree with no `camera_link`, `camera_joint`, `camera_optical_frame`, or `camera_gz_frame` anywhere.

- [x] **Step 1: Delete the old camera from the head**

In `body_head_platform.urdf.xacro` delete the whole `<link name="camera_link">...</link>` block (the link with mesh `camera_link.STL`) and the whole `<joint name="camera_joint" ...>...</joint>` block.

- [x] **Step 2: Delete the retired sensor macro and its no-gravity entry**

```bash
git rm ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro
```

In `ros2_rm_robot/dual_rm_simulation/urdf/gazebo/sim_gazebo.urdf.xacro`, delete the line:
```xml
        <xacro:_gz_no_gravity link_name="camera_link"/>
```
(No replacement: `zed_camera_center` is fixed-jointed to `head_link2`, which Gz lumps; the head links already carry the no-gravity tags.)

- [x] **Step 3: Update the MoveIt SRDFs**

The SRDFs pair with the same core description (their config URDF includes
`$(find dual_rm_description)/urdf/r2d3_description.urdf.xacro`), and
`zed_camera_center` is the only new link with collision geometry:

```bash
sed -i 's/link1="camera_link"/link1="zed_camera_center"/g; s/link2="camera_link"/link2="zed_camera_center"/g' \
  ros2_rm_robot/dual_rm_moveit_config/dual_rm_65b_moveit_config/config/dual_rm_65b_description.srdf \
  ros2_rm_robot/dual_rm_moveit_config/dual_rm_75b_moveit_config/config/dual_rm_75b_description.srdf
grep -c zed_camera_center ros2_rm_robot/dual_rm_moveit_config/dual_rm_65b_moveit_config/config/dual_rm_65b_description.srdf
```
Expected: 19 per file.

- [x] **Step 4: Sweep remaining references**

```bash
grep -rn "camera_link\|camera_optical_frame\|camera_gz_frame\|/camera/" \
  --include="*.py" --include="*.xacro" --include="*.yaml" --include="*.rviz" --include="*.md" \
  ros2_rm_robot r2d3_mujoco ros2_r2d3_apps simulation_quickstart*.md 2>/dev/null | \
  grep -v ros2_realsense2 | grep -v dual_rm_gazebo | grep -v legacy | grep -v "docs/"
```
Fix every hit (quickstart docs: replace the `/camera/*` bridge-table rows with the `/zed/zed_node/*` set; any rviz topic/frame references → new names). `dual_rm_gazebo/` and `dual_rm_description/urdf/legacy/` are self-contained legacy trees — leave them. Re-run until the only remaining hits are legacy/realsense.

- [x] **Step 5: Rebuild everything touched, run all tests**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation r2d3_mujoco dual_rm_65b_moveit_config dual_rm_75b_moveit_config && source install/setup.bash
xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b | grep -c "camera_link\|camera_optical_frame\|camera_gz_frame"
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v
```
Expected: grep count `0` (xacro still exits 0); all tests PASS. Repeat the xacro check for `r2d3_mujoco.urdf.xacro` and the 75b variant.

- [x] **Step 6: Commit**

```bash
git add -A ros2_rm_robot r2d3_mujoco simulation_quickstart_gz.md simulation_quickstart_mujoco.md simulation_quickstart.md
git commit -m "refactor!: remove legacy head camera_link/camera_optical_frame (replaced by ZED 2)"
```

---

### Task 6: RTAB-Map remaps + sim-ready camera gate

**Files:**
- Modify: `ros2_rm_robot/dual_rm_navigation/launch/rtabmap.launch.py`
- Modify: `ros2_rm_robot/dual_rm_navigation/launch/rtabmap_depth_only.launch.py` (same remap pattern)
- Modify: `r2d3_mujoco/scripts/wait_for_sim_ready.py`
- Modify: `r2d3_mujoco/test/test_wait_for_sim_ready.py`

**Interfaces:**
- Consumes: `/zed/zed_node/left/*` + depth topics (Tasks 3–4).
- Produces: `signals_ready(got_scan, got_camera, odom_tf_ok, laser_tf_ok)` and `missing_signal_names(..., camera_topic=...)` (new signatures).

- [x] **Step 1: Update the failing gate tests first**

In `r2d3_mujoco/test/test_wait_for_sim_ready.py`, replace both test classes with:

```python
class TestSignalsReady(unittest.TestCase):
    def test_all_four_required(self):
        self.assertTrue(gate.signals_ready(True, True, True, True))

    def test_any_missing_is_not_ready(self):
        self.assertFalse(gate.signals_ready(False, True, True, True))  # no scan
        self.assertFalse(gate.signals_ready(True, False, True, True))  # no camera
        self.assertFalse(gate.signals_ready(True, True, False, True))  # no odom TF
        self.assertFalse(gate.signals_ready(True, True, True, False))  # no laser TF
        self.assertFalse(gate.signals_ready(False, False, False, False))

    def test_scan_alone_is_not_ready(self):
        # Guard against regressing to a scan-only gate: /scan can flow before
        # the diff-drive odom TF exists, which would start SLAM too early.
        self.assertFalse(gate.signals_ready(True, False, False, False))


class TestMissingSignalNames(unittest.TestCase):
    def test_lists_only_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=True, odom_tf_ok=False, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, ["TF odom->base_footprint"])

    def test_camera_missing_is_named(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=False, odom_tf_ok=True, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, ["/zed/zed_node/left/image_rect_color"])

    def test_all_missing(self):
        missing = gate.missing_signal_names(
            got_scan=False, got_camera=False, odom_tf_ok=False, laser_tf_ok=False,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(
            missing,
            ["/scan", "/zed/zed_node/left/image_rect_color",
             "TF odom->base_footprint", "TF base_footprint->laser_link"])

    def test_none_missing(self):
        missing = gate.missing_signal_names(
            got_scan=True, got_camera=True, odom_tf_ok=True, laser_tf_ok=True,
            odom_frame="odom", base_frame="base_footprint", laser_frame="laser_link",
            camera_topic="/zed/zed_node/left/image_rect_color")
        self.assertEqual(missing, [])
```

Run: `python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_wait_for_sim_ready.py -v` → Expected: FAIL (wrong arity).

- [x] **Step 2: Update wait_for_sim_ready.py**

Replace `signals_ready` and `missing_signal_names`:

```python
def signals_ready(got_scan, got_camera, odom_tf_ok, laser_tf_ok):
    """True only when every readiness signal is present: /scan proves the
    lidar pipeline, the camera image proves the ZED sim camera pipeline,
    odom->base_footprint proves diff-drive odometry, and
    base_footprint->laser_link proves robot_state_publisher. SLAM needs all
    of them connected to build a map -> odom transform."""
    return bool(got_scan and got_camera and odom_tf_ok and laser_tf_ok)


def missing_signal_names(got_scan, got_camera, odom_tf_ok, laser_tf_ok,
                         odom_frame, base_frame, laser_frame, camera_topic):
    """Human-readable list of the signals still missing (for the timeout log)."""
    missing = []
    if not got_scan:
        missing.append("/scan")
    if not got_camera:
        missing.append(camera_topic)
    if not odom_tf_ok:
        missing.append(f"TF {odom_frame}->{base_frame}")
    if not laser_tf_ok:
        missing.append(f"TF {base_frame}->{laser_frame}")
    return missing
```

In `SimReadyGate.__init__`, add parameter `camera_topic` (after `scan_topic`), store `self._camera_topic = camera_topic`, initialize `self._got_camera = camera_topic == ""` (empty string disables the check), and when enabled add:

```python
        from sensor_msgs.msg import Image  # move to top-level imports
        if camera_topic:
            self._camera_sub = self.create_subscription(
                Image, camera_topic, self._on_camera, 1)
```
with callback:
```python
    def _on_camera(self, _msg):
        if not self._got_camera:
            self.get_logger().info("First camera image received.")
        self._got_camera = True
```
Update `_check()` to pass `self._got_camera` into both functions (and `camera_topic=self._camera_topic` into `missing_signal_names`), and update the startup log line to mention the camera topic. In `main()` add:
```python
    parser.add_argument("--camera-topic",
                        default="/zed/zed_node/left/image_rect_color",
                        help="camera image readiness topic ('' disables)")
```
and pass it through to `SimReadyGate`.

- [x] **Step 3: Run tests**

```bash
cd ~/code/r2d3 && colcon build --packages-select r2d3_mujoco && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_wait_for_sim_ready.py -v
```
Expected: all PASS.

- [x] **Step 4: RTAB-Map remaps**

In `ros2_rm_robot/dual_rm_navigation/launch/rtabmap.launch.py` replace the remappings block with:

```python
    # Topic remappings: ZED (sim or real wrapper) → RTAB-Map expected names.
    # Pinned to the LEFT eye: depth is registered to the left eye, so image,
    # camera_info and depth all share zed_left_camera_frame_optical. Never
    # feed RTAB-Map the rgb/ alias or the double-width stereo/ image.
    remappings = [
        ('rgb/image', '/zed/zed_node/left/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/left/camera_info'),
        ('depth/image', '/zed/zed_node/depth/depth_registered'),
        ('scan', '/scan'),
        ('odom', '/diff_drive_controller/odom'),
    ]
```

Apply the same three camera remap lines in `rtabmap_depth_only.launch.py` (keep its other remaps untouched).

- [x] **Step 5: Rebuild + full static sweep**

```bash
cd ~/code/r2d3 && colcon build --packages-select dual_rm_navigation r2d3_mujoco && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v
grep -rn "/camera/" ros2_rm_robot/dual_rm_navigation/ | grep -v ".git"
```
Expected: tests PASS; grep returns nothing.

- [x] **Step 6: Commit**

```bash
git add ros2_rm_robot/dual_rm_navigation/launch/rtabmap.launch.py \
        ros2_rm_robot/dual_rm_navigation/launch/rtabmap_depth_only.launch.py \
        r2d3_mujoco/scripts/wait_for_sim_ready.py r2d3_mujoco/test/test_wait_for_sim_ready.py
git commit -m "feat(nav): RTAB-Map consumes ZED left eye; sim-ready gate checks camera"
```

---

### Task 7: Vendor zed-ros2-wrapper under ros2_zed/

**Files:**
- Create: `ros2_zed/zed-ros2-wrapper/` (vendored, COLCON_IGNOREd)
- Create: `ros2_zed/zed-ros2-interfaces/` (vendored `zed_msgs`, builds everywhere)
- Create: `ros2_zed/README.md`

**Interfaces:**
- Produces: `zed_wrapper` package share (`launch/zed_camera.launch.py`) used by Task 8's bringup launch on the robot.

- [x] **Step 1: Clone and strip**

```bash
mkdir -p ros2_zed && cd ros2_zed
git clone --depth 1 https://github.com/stereolabs/zed-ros2-wrapper.git
git clone --depth 1 https://github.com/stereolabs/zed-ros2-interfaces.git
(cd zed-ros2-wrapper && git rev-parse HEAD) > .zed-ros2-wrapper.sha
(cd zed-ros2-interfaces && git rev-parse HEAD) > .zed-ros2-interfaces.sha
rm -rf zed-ros2-wrapper/.git zed-ros2-interfaces/.git
touch zed-ros2-wrapper/COLCON_IGNORE
cd ..
```

- [x] **Step 2: Write ros2_zed/README.md**

```markdown
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
```

- [x] **Step 3: Verify the dev-machine build story**

```bash
cd ~/code/r2d3 && colcon build --packages-select zed_msgs
colcon list 2>/dev/null | grep -i zed
```
Expected: `zed_msgs` builds; `zed_wrapper`/`zed_components` absent from `colcon list` (COLCON_IGNOREd).

- [x] **Step 4: Commit**

```bash
git add ros2_zed
git commit -m "feat(zed): vendor zed-ros2-wrapper (COLCON_IGNORE) + zed_msgs under ros2_zed/"
```
(If the vendored tree is large this is expected — it mirrors ros2_realsense2.)

---

### Task 8: Real-robot bringup launch

**Files:**
- Create: `ros2_r2d3_apps/r2d3_bringup/launch/zed2.launch.py`
- Create: `ros2_r2d3_apps/r2d3_bringup/config/zed2_params.yaml`
- Modify: `ros2_r2d3_apps/r2d3_bringup/CMakeLists.txt` (only if `config/` isn't already installed)

**Interfaces:**
- Consumes: vendored `zed_wrapper` share (Task 7; robot only).
- Produces: `ros2 launch r2d3_bringup zed2.launch.py` → real `/zed/zed_node/...` topics matching the sim contract.

- [x] **Step 1: Write the params override**

Create `ros2_r2d3_apps/r2d3_bringup/config/zed2_params.yaml`:

```yaml
# ZED 2 overrides for the R2D3 head camera (applied on top of the wrapper's
# zed2.yaml defaults). Keep this minimal - defaults are good.
/zed/zed_node:
  ros__parameters:
    general:
      grab_resolution: 'HD720'   # 1280x720, matches the sim sensors
      pub_resolution: 'NATIVE'
      grab_frame_rate: 15        # matches the sim update_rate
    depth:
      depth_mode: 'NEURAL'
    pos_tracking:
      # Nav owns TF (map->odom->base_footprint); the camera must not publish
      # its own odom/pose TF.
      pos_tracking_enabled: false
```

- [x] **Step 2: Write the launch file**

Create `ros2_r2d3_apps/r2d3_bringup/launch/zed2.launch.py`:

```python
"""Real-robot ZED 2 bringup.

Requires the vendored wrapper to be enabled (robot only):
    rm ros2_zed/zed-ros2-wrapper/COLCON_IGNORE && colcon build --packages-up-to zed_wrapper

publish_tf/publish_urdf are false: robot_state_publisher owns the ZED frames
via the zed2 macro in dual_rm_description; zed_node must not double-publish.
The resulting topics/frames are byte-for-byte the sim contract
(/zed/zed_node/..., zed_left_camera_frame_optical, ...), so RTAB-Map & co.
run unchanged against sim or hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('r2d3_bringup')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')  # robot only

    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed',
            'publish_tf': 'false',
            'publish_urdf': 'false',
            'ros_params_override_path':
                os.path.join(bringup_dir, 'config', 'zed2_params.yaml'),
        }.items(),
    )

    return LaunchDescription([zed_camera])
```

- [x] **Step 3: Ensure config/ + launch/ are installed**

Check `ros2_r2d3_apps/r2d3_bringup/CMakeLists.txt` — if its `install(DIRECTORY ...)` already covers `launch` and `config`, no change. Otherwise extend it to:

```cmake
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
```

- [x] **Step 4: Build + syntax-check**

```bash
cd ~/code/r2d3 && colcon build --packages-select r2d3_bringup && source install/setup.bash
python3 -m py_compile src/R2D3_ros2/ros2_r2d3_apps/r2d3_bringup/launch/zed2.launch.py && echo OK
ls install/r2d3_bringup/share/r2d3_bringup/config/zed2_params.yaml
```
Expected: `OK`; params file installed. (A full `ros2 launch` dry-run needs `zed_wrapper` built — robot only; do not attempt here.)

- [x] **Step 5: Commit**

```bash
git add ros2_r2d3_apps/r2d3_bringup
git commit -m "feat(bringup): real-robot ZED 2 launch (wrapper TF off, URDF owns frames)"
```

---

### Task 9: Full verification sweep

**Files:** none new — verification only.

- [x] **Step 1: Clean full build**

```bash
cd ~/code/r2d3 && colcon build && source install/setup.bash
```
Expected: all packages build; `zed_wrapper` absent (ignored).

- [x] **Step 2: All unit/static tests**

```bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/ src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ -v
```
Expected: all PASS, none skipped.

- [x] **Step 3: Flatten every URDF entrypoint**

```bash
for m in 65b 75b; do
  xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=$m > /dev/null && echo "gz $m OK"
  xacro src/R2D3_ros2/r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro arm_model:=$m > /dev/null && echo "mj $m OK"
done
```
Expected: four OKs.

- [x] **Step 4 (best-effort): Live MuJoCo smoke test**

MuJoCo is the reliable sim on this machine (Gz headless rendering is a
coin-flip — EGL/driver issues; don't treat a Gz render failure as a
regression). Launch headless, then in a second terminal:

```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py headless:=true &
sleep 25
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/left/image_rect_color --window 10 &
sleep 8; kill %2
ros2 topic echo /zed/zed_node/stereo/image_rect_color --once --field width
ros2 topic echo /zed/zed_node/left/image_rect_color --once --field header.frame_id
kill %1
```
Expected: all §1 topics listed; left image ~15 Hz; stereo width `2560`;
frame_id `zed_left_camera_frame_optical`. If the sim itself fails to start
for environment reasons, note it and rely on the static suite.

- [x] **Step 5: Update memory/docs notes and finish**

- Mark the plan checkboxes done.
- Commit any stragglers.
- Use superpowers:finishing-a-development-branch to decide merge/PR next steps.

---

## Self-Review Notes (already applied)

- Spec §1 topic set ↔ Task 3 bridge remaps + Task 2 node pubs + Task 4 ros2_control topics: all nine contract topics covered.
- Frame names consistent everywhere: `zed_left_camera_frame_optical` (wrapper ≥ v5 order).
- Task 4 Step 2 documents why the MuJoCo frame test passes early (frames land in Task 1) — the real gate for Task 4 is the MJCF camera/sensor rename verified in Step 7.
- `stereo_concat` consumes topics that exist in both sims; exact-time sync OK because each sim stamps both eyes identically.
- SRDF: `zed_camera_center` is the only ZED link with collision geometry — correct swap target for all 19 `camera_link` pairs per file.
