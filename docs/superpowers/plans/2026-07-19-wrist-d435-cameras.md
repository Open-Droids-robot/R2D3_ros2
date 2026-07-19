# Wrist D435 Cameras Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a RealSense D435 to each wrist of the R2D3 robot, publishing under `/left_wrist/**` and `/right_wrist/**` in both the Gazebo Harmonic and MuJoCo sims, with pan/tilt aim adjustable from one YAML file.

**Architecture:** Camera frames are defined once in `dual_rm_description` (a frame-only xacro macro, no geometry and no mass — the housing is already part of the wrist link mesh). Mount pose and aim angles come from `config/wrist_cameras.yaml`, loaded via `xacro.load_yaml` and keyed by `arm_model`. Each sim then adds only its own sensor block on top of those shared frames: Gz `<sensor type="rgbd_camera">`, MuJoCo MJCF `<camera>` + `ros2_control` `<sensor>`.

**Tech Stack:** ROS 2, xacro, Gazebo Harmonic (Gz Sim 8) + `ros_gz_bridge`, MuJoCo via `mujoco_ros2_control`, `depth_image_proc`, Python `unittest` + numpy for URDF-geometry tests.

**Spec:** `docs/superpowers/specs/2026-07-19-wrist-d435-cameras-design.md`

## Global Constraints

- **Never orient a sensor with a Gz `<sensor><pose>`.** Orientation comes only from the frame the sensor is mounted on. This is the issue #11 postmortem rule, documented in `zed2_sim.urdf.xacro` and enforced by `test_gz_camera_bore.py`. A `<pose>` override would silently desync Gz from MuJoCo.
- **This repo does not use `--symlink-install`.** `$(find ...)` resolves to the *install* space. After editing any xacro/YAML you MUST run the rebuild command below before any test or launch result is meaningful. Skipping this produces false passes and false failures.
  ```bash
  cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation r2d3_mujoco && source install/setup.bash
  ```
- **Never name a xacro property `e`** — `e` is Euler's number in xacro's expression namespace; it emits `warning: redefining global symbol: e` and can silently misbehave. (Verified during planning.)
- **`docs/` is in `.gitignore`** but every existing spec/plan is tracked. Use `git add -f` for files under `docs/`.
- Sim camera update rate: **15 Hz**, matching the ZED sim.
- Frame naming: `{side}_wrist_camera_*` where `{side}` is `left` or `right`. Topic namespace: `/{side}_wrist/`.

## D435 constants (use these exact values)

| property | value | note |
| --- | --- | --- |
| horizontal FOV | `1.5184` rad | 87°, D435 depth FOV |
| resolution | `848 × 480` | native D435 depth mode |
| depth clip | `0.3` – `3.0` m | |
| MuJoCo `fovy` | `56.4` deg | `2*atan(tan(1.5184/2) * 480/848)` = 0.9846 rad |
| update rate | `15` Hz | |

## Mount geometry (measured from the meshes during planning — do not re-derive)

The housing is an existing rectangular bar in each wrist link mesh: outer face at `|x| = 0.0783`, ~22 mm thick, spanning `y = ±0.0565`. The mount frame is the bar **centroid** (`|x| = 0.0671`), so pan/tilt pivot the camera in place inside the housing instead of swinging it through a wall.

The X sign is **not** a left/right rule — 65b-left is `+X`, the other three are `−X`. This is why it is stored as explicit per-entry data rather than derived.

## File Structure

| file | responsibility |
| --- | --- |
| `ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml` | **new** — mount pose + aim, per (arm_model, side). The single source of truth for both sims. |
| `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/d435.urdf.xacro` | **new** — frame-only D435 macro (mount joint, aim joint, optical frames). Sibling of `zed2.urdf.xacro`. |
| `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/r2d3_description.urdf.xacro` | instantiate both wrist cameras from the YAML |
| `ros2_rm_robot/dual_rm_description/dual_rm_description/CMakeLists.txt` | install the new `config/` dir |
| `ros2_rm_robot/dual_rm_simulation/urdf/sensors/wrist_cams_sim.urdf.xacro` | **new** — Gz `rgbd_camera` blocks |
| `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro` | include + instantiate the Gz macro |
| `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py` | bridge + remap to realsense topic names |
| `ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py` | **new** — mount pose / aim / variant coverage |
| `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py` | extend with wrist-camera bore assertions |
| `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro` | MJCF `<camera>` entries |
| `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro` | `ros2_control` `<sensor>` entries |
| `r2d3_mujoco/launch/mujoco_sim.launch.py` | pointcloud containers for both wrists |
| `r2d3_mujoco/test/test_camera_optical_frame.py` | extend with wrist-camera bore assertions |

---

### Task 1: Config + frame macro in the shared description

This task delivers the frames themselves. Both sims depend on it; nothing renders yet.

**Files:**
- Create: `ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml`
- Create: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/d435.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/r2d3_description.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_description/dual_rm_description/CMakeLists.txt:10`
- Test: `ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py`

**Interfaces:**
- Produces:
  - YAML at `$(find dual_rm_description)/config/wrist_cameras.yaml`, top-level keys `65b` / `75b`, each with `left` / `right`, each entry having keys `parent` (str), `xyz` (3 floats), `rpy` (3 floats), `pan` (float), `tilt` (float).
  - Macro `d435_camera` with params `name parent pan tilt *origin`.
  - Links, for `name` in `{left_wrist, right_wrist}`: `{name}_camera_mount`, `{name}_camera_link`, `{name}_camera_color_frame`, `{name}_camera_color_optical_frame`, `{name}_camera_depth_frame`, `{name}_camera_depth_optical_frame`.
  - Joints: `{name}_camera_mount_joint`, `{name}_camera_aim_joint`, `{name}_camera_color_joint`, `{name}_camera_color_optical_joint`, `{name}_camera_depth_joint`, `{name}_camera_depth_optical_joint`.

- [ ] **Step 1: Write the config YAML**

Create `ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml`:

```yaml
# Wrist-mounted RealSense D435 mount + aim, per arm variant and side.
#
# The camera sits INSIDE the rectangular housing that is already part of each
# wrist link's mesh (no extra geometry is added by the URDF). `xyz` is that
# housing's centroid, so pan/tilt pivot the camera in place rather than
# swinging it out through a housing wall.
#
#   xyz  - housing centroid in the parent wrist link's frame (m). Measured
#          from the meshes: outer face at |x|=0.0783, bar ~22mm thick.
#   rpy  - nominal outward bore. NOTE the X offset sign is NOT a left/right
#          rule (65b-left is +X; the other three are -X), so the yaw that
#          turns the bore outward is stored explicitly per entry, never
#          derived from the side.
#   pan  - rotation about the mount Z, radians. Left/right sweep.
#   tilt - rotation about the mount Y, radians. NEGATIVE = downward, toward
#          the gripper. This is the knob you normally want.
#
# pan: 0, tilt: 0 == camera perpendicular to the wrist, boring straight out
# along the housing normal. Both sims read this one file, so editing here
# re-aims the camera in Gazebo and MuJoCo identically.

65b:
  left:
    parent: l_link6
    xyz: [0.0671, 0.0, 0.0275]
    rpy: [0.0, 0.0, 0.0]
    pan: 0.0
    tilt: 0.0
  right:
    parent: r_link6
    xyz: [-0.0671, 0.0, 0.0301]
    rpy: [0.0, 0.0, 3.14159265]
    pan: 0.0
    tilt: 0.0

75b:
  left:
    parent: l_link7
    xyz: [-0.0671, 0.0, 0.0275]
    rpy: [0.0, 0.0, 3.14159265]
    pan: 0.0
    tilt: 0.0
  right:
    parent: r_link7
    xyz: [-0.0671, 0.0, 0.0256]
    rpy: [0.0, 0.0, 3.14159265]
    pan: 0.0
    tilt: 0.0
```

- [ ] **Step 2: Write the failing test**

Create `ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py`:

```python
"""Guards the wrist D435 mount frames against the two mistakes this geometry
invites.

1. MIRRORED SIGN. The housing X offset is not a left/right rule: 65b-left sits
   at +X while 65b-right, 75b-left and 75b-right all sit at -X. Deriving the
   sign from the side instead of reading it from the config buries a camera
   inside its own wrist link, where it renders the link interior and looks
   merely "dark" rather than obviously broken.
2. AIM APPLIED AT THE WRONG PIVOT. pan/tilt must rotate about the housing
   centroid. If the aim joint is folded into the mount joint, tilting also
   translates the camera out through the housing wall.

The mount pose is asserted against config/wrist_cameras.yaml itself, so the
YAML stays the single source of truth: editing it cannot silently drift from
what the URDF builds.

NOTE: the xacro include resolves via $(find ...) -> the INSTALL space.
Rebuild (colcon build --packages-select dual_rm_description
dual_rm_simulation) after editing xacros or the YAML, or this test sees old
files.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np
import yaml

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"


def _config_path():
    out = subprocess.run(
        ["ros2", "pkg", "prefix", "--share", "dual_rm_description"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise unittest.SkipTest("dual_rm_description not found (workspace not built?)")
    return Path(out.stdout.strip()) / "config" / "wrist_cameras.yaml"


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis: Rz*Ry*Rx


def _flatten_urdf(arm_model):
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        raise unittest.SkipTest("xacro not on PATH (source the ROS workspace)")
    out = subprocess.run(
        [xacro_bin, str(XACRO), f"arm_model:={arm_model}"],
        capture_output=True, text=True,
    )
    if out.returncode != 0:
        raise unittest.SkipTest(f"xacro failed (workspace not built?): {out.stderr[-400:]}")
    return out.stdout


def _joints(urdf_str):
    """child_link -> (parent_link, rpy, xyz) for every joint."""
    dom = minidom.parseString(urdf_str)
    out = {}
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
        out[ch[0].getAttribute("link")] = (
            par[0].getAttribute("link"), rpy, np.array(xyz))
    return out


def _links(urdf_str):
    dom = minidom.parseString(urdf_str)
    return {ln.getAttribute("name") for ln in dom.getElementsByTagName("link")}


class TestWristCameraMount(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load(_config_path().read_text())
        cls.urdf = {m: _flatten_urdf(m) for m in ("65b", "75b")}

    def test_all_frames_exist_for_both_variants(self):
        for model in ("65b", "75b"):
            links = _links(self.urdf[model])
            for side in ("left", "right"):
                for suffix in ("camera_mount", "camera_link",
                               "camera_color_frame", "camera_color_optical_frame",
                               "camera_depth_frame", "camera_depth_optical_frame"):
                    name = f"{side}_wrist_{suffix}"
                    self.assertIn(name, links, f"{model}: missing link {name}")

    def test_mount_matches_config_exactly(self):
        """Catches the mirrored-sign failure: a camera at the wrong X sign is
        inside the wrist link, not outside it."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                parent, rpy, xyz = joints[f"{side}_wrist_camera_mount"]
                self.assertEqual(parent, entry["parent"],
                                 f"{model}/{side}: wrong parent link")
                np.testing.assert_allclose(
                    xyz, entry["xyz"], atol=1e-9,
                    err_msg=f"{model}/{side}: mount xyz does not match config")
                np.testing.assert_allclose(
                    rpy, entry["rpy"], atol=1e-9,
                    err_msg=f"{model}/{side}: mount rpy does not match config")

    def test_aim_is_a_separate_joint_carrying_only_pan_tilt(self):
        """pan/tilt must live in their own joint so they pivot about the
        housing centroid and never translate the camera."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                parent, rpy, xyz = joints[f"{side}_wrist_camera_link"]
                self.assertEqual(parent, f"{side}_wrist_camera_mount",
                                 f"{model}/{side}: aim joint must hang off the mount")
                np.testing.assert_allclose(
                    xyz, [0.0, 0.0, 0.0], atol=1e-12,
                    err_msg=f"{model}/{side}: aim joint must not translate")
                np.testing.assert_allclose(
                    rpy, [0.0, entry["tilt"], entry["pan"]], atol=1e-9,
                    err_msg=f"{model}/{side}: aim joint rpy should be (0, tilt, pan)")

    def test_bore_points_away_from_the_wrist_axis(self):
        """At the nominal aim, the camera must look OUTWARD. This is the
        end-to-end statement of the mirrored-sign guard: the bore direction
        and the mount offset must share a sign."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                entry = self.cfg[model][side]
                _, rpy, xyz = joints[f"{side}_wrist_camera_mount"]
                bore = _rpy_to_R(*rpy)[:, 0]
                self.assertGreater(
                    float(np.dot(bore, np.array(entry["xyz"]))), 0.0,
                    f"{model}/{side}: bore points back into the wrist link "
                    f"(bore={bore}, offset={entry['xyz']})")

    def test_color_and_depth_optical_frames_are_coincident(self):
        """Gz rgbd_camera renders colour and depth from ONE viewpoint, and the
        real D435 publishes depth aligned to colour. Modelling a baseline here
        would be a lie the sim cannot honour."""
        for model in ("65b", "75b"):
            joints = _joints(self.urdf[model])
            for side in ("left", "right"):
                _, _, c = joints[f"{side}_wrist_camera_color_frame"]
                _, _, d = joints[f"{side}_wrist_camera_depth_frame"]
                np.testing.assert_allclose(
                    c, d, atol=1e-12,
                    err_msg=f"{model}/{side}: colour and depth frames must coincide")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py -v
```

Expected: FAIL — `test_all_frames_exist_for_both_variants` reports `missing link left_wrist_camera_mount`. (If instead every test SKIPs, the workspace is not built — run the rebuild command from Global Constraints first.)

- [ ] **Step 4: Write the D435 frame macro**

Create `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/d435.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!--
  Intel RealSense D435 frame set for the R2D3 wrists.

  FRAMES ONLY - no visual, no collision, no inertial. The camera housing is
  already modelled as part of each wrist link's mesh (l_link6/r_link6 on 65b,
  l_link7/r_link7 on 75b), and the wrist link already carries its mass. Adding
  geometry here would double-count both.

  The chain is deliberately split into two fixed joints:

    <wrist link>
      |- mount joint   hardware: the housing centroid + nominal outward bore.
      |                Never edited by a user.
      {name}_camera_mount
        |- aim joint   the tunable: rpy = (0, tilt, pan) from
        |              config/wrist_cameras.yaml. Pure rotation, so the
        |              camera pivots INSIDE the housing instead of
        |              translating out through a wall.
        {name}_camera_link
          |- {name}_camera_color_frame -> ..._color_optical_frame
          |- {name}_camera_depth_frame -> ..._depth_optical_frame

  Colour and depth frames are COINCIDENT on purpose. Gz's rgbd_camera renders
  both from a single viewpoint, and the real D435 publishes depth aligned to
  colour, so a modelled baseline would be a fiction neither sim nor hardware
  reproduces.

  Optical frames use the standard rpy="-pi/2 0 -pi/2" (REP 103: optical +Z
  forward, +X right, +Y down), matching zed2.urdf.xacro.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="d435_camera" params="name parent pan tilt *origin">

    <!-- Housing centroid + nominal outward bore (hardware geometry) -->
    <link name="${name}_camera_mount"/>
    <joint name="${name}_camera_mount_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${name}_camera_mount"/>
      <xacro:insert_block name="origin"/>
    </joint>

    <!-- Aim: the only user-tunable transform. Pure rotation. -->
    <link name="${name}_camera_link"/>
    <joint name="${name}_camera_aim_joint" type="fixed">
      <parent link="${name}_camera_mount"/>
      <child link="${name}_camera_link"/>
      <origin xyz="0 0 0" rpy="0 ${tilt} ${pan}"/>
    </joint>

    <!-- Colour -->
    <link name="${name}_camera_color_frame"/>
    <joint name="${name}_camera_color_joint" type="fixed">
      <parent link="${name}_camera_link"/>
      <child link="${name}_camera_color_frame"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

    <link name="${name}_camera_color_optical_frame"/>
    <joint name="${name}_camera_color_optical_joint" type="fixed">
      <parent link="${name}_camera_color_frame"/>
      <child link="${name}_camera_color_optical_frame"/>
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
    </joint>

    <!-- Depth (coincident with colour - see header) -->
    <link name="${name}_camera_depth_frame"/>
    <joint name="${name}_camera_depth_joint" type="fixed">
      <parent link="${name}_camera_link"/>
      <child link="${name}_camera_depth_frame"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

    <link name="${name}_camera_depth_optical_frame"/>
    <joint name="${name}_camera_depth_optical_joint" type="fixed">
      <parent link="${name}_camera_depth_frame"/>
      <child link="${name}_camera_depth_optical_frame"/>
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
    </joint>

  </xacro:macro>

</robot>
```

- [ ] **Step 5: Instantiate both cameras in the robot description**

In `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/r2d3_description.urdf.xacro`, add the include next to the other shared includes (after the `body_head_platform` line):

```xml
    <xacro:include filename="$(find dual_rm_description)/urdf/sensors/d435.urdf.xacro"/>
```

Then append this block just before the closing `</robot>` tag:

```xml
    <!-- ═══════════════════════════════════════════════════════════
         Wrist RealSense D435 cameras

         Mount pose and aim (pan/tilt) come from config/wrist_cameras.yaml,
         which both sims read - edit there to re-aim, not here. Frames only;
         the housing is already part of the wrist link meshes.
         ═══════════════════════════════════════════════════════════ -->
    <xacro:property name="wrist_cam_cfg"
                    value="${xacro.load_yaml('$(find dual_rm_description)/config/wrist_cameras.yaml')[arm_model]}"/>

    <xacro:macro name="_wrist_camera" params="side">
      <!-- NOTE: do not name this property 'e' - xacro reserves e (Euler's
           number) and silently redefines it. -->
      <xacro:property name="entry" value="${wrist_cam_cfg[side]}"/>
      <xacro:d435_camera name="${side}_wrist" parent="${entry['parent']}"
                         pan="${entry['pan']}" tilt="${entry['tilt']}">
        <origin xyz="${entry['xyz'][0]} ${entry['xyz'][1]} ${entry['xyz'][2]}"
                rpy="${entry['rpy'][0]} ${entry['rpy'][1]} ${entry['rpy'][2]}"/>
      </xacro:d435_camera>
    </xacro:macro>

    <xacro:_wrist_camera side="left"/>
    <xacro:_wrist_camera side="right"/>
```

- [ ] **Step 6: Install the config directory**

In `ros2_rm_robot/dual_rm_description/dual_rm_description/CMakeLists.txt`, change line 10 from:

```cmake
install(DIRECTORY launch urdf meshes rviz DESTINATION share/${PROJECT_NAME})
```

to:

```cmake
install(DIRECTORY launch urdf meshes rviz config DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 7: Rebuild and run the test to verify it passes**

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py -v
```

Expected: 5 passed.

- [ ] **Step 8: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml \
        ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/sensors/d435.urdf.xacro \
        ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/r2d3_description.urdf.xacro \
        ros2_rm_robot/dual_rm_description/dual_rm_description/CMakeLists.txt \
        ros2_rm_robot/dual_rm_simulation/test/test_wrist_camera_mount.py
git commit -m "feat(description): wrist D435 camera frames, aimed from wrist_cameras.yaml"
```

- [ ] **Step 9: Verify the aim knob actually moves the camera**

This proves the YAML is a live knob, not decoration. It runs *after* the commit so the temporary edit can be reverted with `git checkout` (before the commit the file is untracked and `git checkout` would fail).

Temporarily set `tilt: -0.4` for `65b/left` and confirm the URDF picks it up:

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash
sed -i '0,/tilt: 0.0/s//tilt: -0.4/' src/R2D3_ros2/ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml
colcon build --packages-select dual_rm_description >/dev/null && source install/setup.bash
xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b \
  | grep -A2 'left_wrist_camera_aim_joint'
```

Expected: the `<origin>` shows `rpy="0 -0.4 0"`. Then revert:

```bash
cd /home/samzpc/code/r2d3
git -C src/R2D3_ros2 checkout ros2_rm_robot/dual_rm_description/dual_rm_description/config/wrist_cameras.yaml
colcon build --packages-select dual_rm_description >/dev/null
```

Expected: the `<origin>` under `left_wrist_camera_aim_joint` showed `rpy="0 -0.4 0"`, and `git status` is clean afterwards.

---

### Task 2: Gazebo sensors + topic bridge

**Files:**
- Create: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/wrist_cams_sim.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro`
- Modify: `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py:118-145`
- Test: `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py`

**Interfaces:**
- Consumes: from Task 1 — `{side}_wrist_camera_color_frame`, `{side}_wrist_camera_color_optical_frame`.
- Produces: macro `wrist_cams_sim_sensors` (no params). Gz sensor names `left_wrist` / `right_wrist`; Gz topic prefixes `left_wrist` / `right_wrist`. ROS topics `/{side}_wrist/color/image_raw`, `/color/camera_info`, `/depth/image_rect_raw`, `/depth/color/points`.

- [ ] **Step 1: Write the failing test**

Append to `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py`, before the `if __name__ == "__main__":` block:

```python
class TestGzWristCameraBore(unittest.TestCase):
    """The wrist cameras must render along the direction their mount frame
    bores, and label their output with an optical frame that agrees.

    Unlike the ZED, the wrists hang off moving arm joints, so there is no
    fixed nav-frame expectation to assert. Instead we assert the two
    invariants that survive any arm pose: the sensor must be mounted on the
    camera colour frame (never oriented by a <pose>, per the issue #11
    postmortem), and the rendered optical axes must equal the optical frame
    named in gz_frame_id.
    """

    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        cls.sensors = _gz_camera_sensors(cls.urdf)

    def test_wrist_sensors_mounted_on_colour_frames(self):
        for side in ("left", "right"):
            ref = f"{side}_wrist_camera_color_frame"
            self.assertIn(ref, self.sensors,
                          f"no Gz camera sensor mounted on {ref}")

    def test_wrist_sensor_pose_is_identity(self):
        """Issue #11 postmortem: orientation lives in mount frames, never a
        sensor <pose>. A <pose> here would desync Gz from MuJoCo, which has
        no equivalent override."""
        for side in ("left", "right"):
            _, R_pose = self.sensors[f"{side}_wrist_camera_color_frame"]
            np.testing.assert_allclose(
                R_pose, np.eye(3), atol=1e-12,
                err_msg=f"{side} wrist: sensor <pose> must not rotate the render")

    def test_wrist_render_matches_optical_frame_label(self):
        for side in ("left", "right"):
            ref = f"{side}_wrist_camera_color_frame"
            _, R_pose = self.sensors[ref]
            R_chain, _ = _joint_chain(self.urdf, ref)
            R_optical_render = (R_chain @ R_pose) @ _SENSOR_TO_OPTICAL
            R_optical_label, _ = _joint_chain(
                self.urdf, f"{side}_wrist_camera_color_optical_frame")
            np.testing.assert_allclose(
                R_optical_render, R_optical_label, atol=1e-6,
                err_msg=f"{side} wrist: rendered optical axes disagree with "
                        f"the {side}_wrist_camera_color_optical_frame label")
```

Also update the existing `test_sensors_mounted_on_zed_camera_frames`, which asserts an exact set of all camera sensors and will now be wrong. Change its body from:

```python
        self.assertEqual(
            set(self.sensors.keys()),
            {"zed_left_camera_frame", "zed_right_camera_frame"})
```

to:

```python
        self.assertLessEqual(
            {"zed_left_camera_frame", "zed_right_camera_frame"},
            set(self.sensors.keys()))
        for ref in ("zed_left_camera_frame", "zed_right_camera_frame"):
            _, R_pose = self.sensors[ref]
            np.testing.assert_allclose(
                R_pose, np.eye(3), atol=1e-12,
                err_msg=f"{ref}: sensor <pose> must not rotate the render")
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```

Expected: FAIL — `no Gz camera sensor mounted on left_wrist_camera_color_frame`.

- [ ] **Step 3: Write the Gz sensor macro**

Create `ros2_rm_robot/dual_rm_simulation/urdf/sensors/wrist_cams_sim.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!--
  Wrist RealSense D435 sensors for Gz Sim (Harmonic).

  Each wrist gets one rgbd_camera mounted DIRECTLY on its colour frame, which
  already bores outward along the housing normal with the configured pan/tilt
  applied (dual_rm_description d435.urdf.xacro + config/wrist_cameras.yaml).

  Per the issue #11 postmortem, a sensor is NEVER oriented with a <pose> -
  only via the frame it is mounted on. That rule is load-bearing here: MuJoCo
  has no equivalent to a Gz <pose>, so any orientation expressed that way
  would silently make the two sims disagree. Guarded by
  test_gz_camera_bore.py::TestGzWristCameraBore.

  D435 optics: 87 deg hfov, 848x480, 0.3-3.0 m.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:macro name="_wrist_gz_cam" params="side">
        <gazebo reference="${side}_wrist_camera_color_frame">
            <sensor name="${side}_wrist" type="rgbd_camera">
                <always_on>true</always_on>
                <update_rate>15</update_rate>
                <topic>${side}_wrist</topic>
                <gz_frame_id>${side}_wrist_camera_color_optical_frame</gz_frame_id>

                <camera name="${side}_wrist">
                    <horizontal_fov>1.5184</horizontal_fov>  <!-- 87 deg, D435 -->
                    <image>
                        <width>848</width>
                        <height>480</height>
                        <format>R8G8B8</format>
                    </image>
                    <depth_camera>
                        <clip>
                            <near>0.3</near>
                            <far>3.0</far>
                        </clip>
                    </depth_camera>
                    <clip>
                        <near>0.3</near>
                        <far>3.0</far>
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

    <xacro:macro name="wrist_cams_sim_sensors">
        <xacro:_wrist_gz_cam side="left"/>
        <xacro:_wrist_gz_cam side="right"/>
    </xacro:macro>

</robot>
```

- [ ] **Step 4: Wire it into the sim URDF**

In `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro`, add to the includes block (after the `zed2_sim` include):

```xml
    <xacro:include filename="$(find dual_rm_simulation)/urdf/sensors/wrist_cams_sim.urdf.xacro"/>
```

and in section 3, after the `<xacro:zed2_sim_sensors/>` line:

```xml
    <!-- Wrist D435 cameras (Gz rgbd_camera per wrist) -->
    <xacro:wrist_cams_sim_sensors/>
```

- [ ] **Step 5: Rebuild and run the test to verify it passes**

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_simulation && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```

Expected: all tests pass, including the 3 new `TestGzWristCameraBore` cases.

- [ ] **Step 6: Add the bridge entries**

In `ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py`, add to the `arguments` list of the `bridge` node, after the existing ZED entries (around line 135):

```python
            # Wrist D435s: one rgbd_camera per wrist (topics left_wrist,
            # right_wrist).
            '/left_wrist/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/left_wrist/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/left_wrist/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/left_wrist/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/right_wrist/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/right_wrist/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/right_wrist/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/right_wrist/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
```

and to the `remappings` list, after the existing ZED remappings (around line 143):

```python
            # Gz topic names -> the real realsense2_camera topic contract, so
            # swapping in real D435s is a launch-file change with no consumer
            # edits.
            ('/left_wrist/image', '/left_wrist/color/image_raw'),
            ('/left_wrist/camera_info', '/left_wrist/color/camera_info'),
            ('/left_wrist/depth_image', '/left_wrist/depth/image_rect_raw'),
            ('/left_wrist/points', '/left_wrist/depth/color/points'),
            ('/right_wrist/image', '/right_wrist/color/image_raw'),
            ('/right_wrist/camera_info', '/right_wrist/color/camera_info'),
            ('/right_wrist/depth_image', '/right_wrist/depth/image_rect_raw'),
            ('/right_wrist/points', '/right_wrist/depth/color/points'),
```

- [ ] **Step 7: Verify the topics actually publish**

Static checks cannot prove a sensor renders. Launch the sim and confirm real data:

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_simulation && source install/setup.bash
ros2 launch dual_rm_simulation gz_sim.launch.py &
sleep 45
ros2 topic list | grep wrist
ros2 topic hz /left_wrist/color/image_raw --window 20
ros2 topic hz /right_wrist/color/image_raw --window 20
```

Expected: all 8 wrist topics listed; both `hz` reports settle near 15 Hz.

**If `hz` reports nothing:** check `/clock` is advancing (`ros2 topic hz /clock`). Headless rendering on this machine is unreliable — EGL is a coin flip, and when it fails the clock stalls and every camera goes silent. That is an environment failure, not a code failure. Record it and fall back to the static test results rather than "fixing" the URDF.

Kill the sim when done: `pkill -f 'gz sim'; pkill -f ruby`. Verify with `pgrep -af 'gz sim|ruby'` — these processes survive a naive `pkill` and a surviving stale instance will make the *next* run look broken in confusing ways (stuck joints, spawner latching onto a foreign world).

- [ ] **Step 8: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_simulation/urdf/sensors/wrist_cams_sim.urdf.xacro \
        ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro \
        ros2_rm_robot/dual_rm_simulation/launch/gz_sim.launch.py \
        ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py
git commit -m "feat(sim): wrist D435 Gz sensors bridged to realsense topic contract"
```

---

### Task 3: MuJoCo cameras

**Files:**
- Modify: `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro:177-187`
- Modify: `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro:129-146`
- Test: `r2d3_mujoco/test/test_camera_optical_frame.py`

**Interfaces:**
- Consumes: from Task 1 — `{side}_wrist_camera_color_optical_frame`.
- Produces: MJCF cameras named `left_wrist` / `right_wrist`, publishing the same ROS topics as Task 2.

**Critical:** the MJCF `<camera name=...>` and the `ros2_control` `<sensor name=...>` must be **identical strings**, or the converter silently produces no images. Both are `left_wrist` / `right_wrist`.

- [ ] **Step 1: Write the failing test**

Append to `r2d3_mujoco/test/test_camera_optical_frame.py`, before the `if __name__ == "__main__":` block:

```python
class TestWristCameraOpticalFrames(unittest.TestCase):
    """MuJoCo hangs its cameras on these frames' sites, so the frames must
    exist in the MuJoCo URDF too - the ZED equivalent of this guard is what
    caught the issue #11 yaw error.

    The wrists hang off moving arm joints, so there is no fixed nav-frame
    bore to assert. What must hold in any pose is that the optical frame is a
    proper REP-103 optical frame relative to its camera frame: +Z forward,
    +X right, +Y down.
    """

    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()

    def test_optical_frames_exist(self):
        links = {ln.getAttribute("name")
                 for ln in minidom.parseString(self.urdf).getElementsByTagName("link")}
        for side in ("left", "right"):
            for suffix in ("camera_color_frame", "camera_color_optical_frame"):
                name = f"{side}_wrist_{suffix}"
                self.assertIn(name, links, f"missing link {name}")

    def test_optical_rotation_is_rep103(self):
        for side in ("left", "right"):
            R_cam, _ = _joint_chain(self.urdf, f"{side}_wrist_camera_color_frame")
            R_opt, _ = _joint_chain(self.urdf,
                                    f"{side}_wrist_camera_color_optical_frame")
            # optical axes expressed in the camera frame
            R_rel = R_cam.T @ R_opt
            expected = np.array([[0.0, 0.0, 1.0],
                                 [-1.0, 0.0, 0.0],
                                 [0.0, -1.0, 0.0]])
            np.testing.assert_allclose(
                R_rel, expected, atol=1e-6,
                err_msg=f"{side} wrist: optical frame is not REP-103 "
                        f"(+Z fwd, +X right, +Y down)")
```

Confirm `minidom` is imported at the top of the file (it is, per the existing `from xml.dom import minidom`).

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py -v
```

Expected: FAIL — `missing link left_wrist_camera_color_frame`. (The MuJoCo URDF includes `r2d3_description`, so Task 1's frames appear here once `r2d3_mujoco` is rebuilt. If it fails for another reason, rebuild first.)

- [ ] **Step 3: Add the MJCF cameras**

In `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`, inside `<processed_inputs>` after the two ZED `<camera>` entries (around line 186):

```xml
                <!-- Wrist D435s at the colour optical-frame sites. fovy(deg)
                     derived from the Gazebo horizontal_fov of 1.5184 rad at
                     848x480: fovy = 2*atan(tan(1.5184/2)*480/848) = 56.4 deg.
                     Camera names must equal the ros2_control <sensor> names in
                     mujoco_ros2_control.urdf.xacro. -->
                <camera site="left_wrist_camera_color_optical_frame" name="left_wrist"
                        fovy="56.4" mode="fixed" resolution="848 480"/>
                <camera site="right_wrist_camera_color_optical_frame" name="right_wrist"
                        fovy="56.4" mode="fixed" resolution="848 480"/>
```

- [ ] **Step 4: Add the ros2_control sensors**

In `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro`, after the `zed_right` `</sensor>` (around line 146):

```xml
            <!-- Wrist D435s: sensor names must equal the MJCF camera names
                 (mujoco_inputs.urdf.xacro). Topic names match the real
                 realsense2_camera contract, same as the Gz bridge. -->
            <sensor name="left_wrist">
                <param name="frame_name">left_wrist_camera_color_optical_frame</param>
                <param name="info_topic">/left_wrist/color/camera_info</param>
                <param name="image_topic">/left_wrist/color/image_raw</param>
                <param name="depth_topic">/left_wrist/depth/image_rect_raw</param>
            </sensor>
            <sensor name="right_wrist">
                <param name="frame_name">right_wrist_camera_color_optical_frame</param>
                <param name="info_topic">/right_wrist/color/camera_info</param>
                <param name="image_topic">/right_wrist/color/image_raw</param>
                <param name="depth_topic">/right_wrist/depth/image_rect_raw</param>
            </sensor>
```

- [ ] **Step 5: Rebuild and run the test to verify it passes**

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select r2d3_mujoco && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py -v
```

Expected: all tests pass, including the 2 new `TestWristCameraOpticalFrames` cases.

- [ ] **Step 6: Verify the MJCF actually converts**

The converter silently drops a `<camera site=...>` whose site does not exist, so confirm the cameras survive into the generated MJCF:

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_ensure_mjcf.py -v
```

Expected: PASS. Then launch and confirm images:

```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py headless:=true &
sleep 45
ros2 topic hz /left_wrist/color/image_raw --window 20
ros2 topic hz /right_wrist/color/image_raw --window 20
```

Expected: both settle near 15 Hz. Kill with `pkill -f mujoco` when done.

- [ ] **Step 7: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git add r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro \
        r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro \
        r2d3_mujoco/test/test_camera_optical_frame.py
git commit -m "feat(mujoco): wrist D435 cameras on the shared optical frames"
```

---

### Task 4: MuJoCo wrist pointclouds

Gz publishes `points` natively; MuJoCo does not, so it needs `depth_image_proc` to complete the `/depth/color/points` half of the topic contract on both sims.

**Files:**
- Modify: `r2d3_mujoco/launch/mujoco_sim.launch.py:127-148`

**Interfaces:**
- Consumes: from Task 3 — `/{side}_wrist/color/camera_info`, `/color/image_raw`, `/depth/image_rect_raw`.
- Produces: `/{side}_wrist/depth/color/points`.

- [ ] **Step 1: Add the pointcloud containers**

In `r2d3_mujoco/launch/mujoco_sim.launch.py`, after the existing `pointcloud_container` definition (around line 148), add:

```python
    # -- /{side}_wrist/depth/color/points from each wrist D435 --
    # Gz publishes `points` natively from its rgbd_camera; MuJoCo does not, so
    # depth_image_proc completes the same topic contract on this sim.
    def _wrist_points(side):
        return ComposableNodeContainer(
            name=f"{side}_wrist_points_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb",
                    parameters=[{"use_sim_time": True}],
                    remappings=[
                        ("rgb/camera_info", f"/{side}_wrist/color/camera_info"),
                        ("rgb/image_rect_color", f"/{side}_wrist/color/image_raw"),
                        ("depth_registered/image_rect", f"/{side}_wrist/depth/image_rect_raw"),
                        ("points", f"/{side}_wrist/depth/color/points"),
                    ],
                ),
            ],
            output="screen",
        )

    left_wrist_points = _wrist_points("left")
    right_wrist_points = _wrist_points("right")
```

- [ ] **Step 2: Add them to the returned launch list**

In the `return [` list at the end of the same function, add `left_wrist_points,` and `right_wrist_points,` alongside `pointcloud_container`.

- [ ] **Step 3: Verify the pointclouds publish**

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select r2d3_mujoco && source install/setup.bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py headless:=true &
sleep 45
ros2 topic hz /left_wrist/depth/color/points --window 10
ros2 topic hz /right_wrist/depth/color/points --window 10
```

Expected: both publish (rate may be below 15 Hz — `depth_image_proc` drops frames when colour and depth stamps do not pair, which is acceptable here).

**If a topic is silent while `/depth/image_rect_raw` is publishing:** this is a stamp-pairing failure, not a wiring failure. `depth_image_proc` uses an exact-time sync by default; if the sim stamps colour and depth even a millisecond apart the pairing starves. Check with `ros2 topic echo --field header.stamp` on both inputs before changing any remapping.

Kill with `pkill -f mujoco` when done.

- [ ] **Step 4: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git add r2d3_mujoco/launch/mujoco_sim.launch.py
git commit -m "feat(mujoco): wrist D435 pointclouds via depth_image_proc"
```

---

### Task 5: Document the aim knob

The whole point of the YAML is that a user can find and turn it. Undocumented, it is invisible.

**Files:**
- Modify: `simulation_quickstart_gz.md`
- Modify: `simulation_quickstart_mujoco.md`

- [ ] **Step 1: Add an aiming section to both quickstarts**

Append to both `simulation_quickstart_gz.md` and `simulation_quickstart_mujoco.md`:

Note the outer fence below is FOUR backticks because the content itself contains
fenced blocks — copy only what is inside it, not the outer fence.

````markdown
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

`pan: 0, tilt: 0` is the camera perpendicular to the wrist, boring straight
out along the housing normal.

Rebuild after editing — this workspace does not use `--symlink-install`, so
xacro reads the installed copy:

```bash
colcon build --packages-select dual_rm_description && source install/setup.bash
```
````

- [ ] **Step 2: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2
git add simulation_quickstart_gz.md simulation_quickstart_mujoco.md
git commit -m "docs: how to aim the wrist D435 cameras"
```

---

## Final verification

- [ ] Run the full affected test set:

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_description dual_rm_simulation r2d3_mujoco && source install/setup.bash
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/ src/R2D3_ros2/r2d3_mujoco/test/ -v
```

Expected: all pass. Report any SKIPs explicitly — a skip is not a pass, and the tests skip themselves when the workspace is unbuilt.

- [ ] Confirm both variants still flatten:

```bash
for m in 65b 75b; do
  xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=$m > /dev/null && echo "gz $m OK"
  xacro src/R2D3_ros2/r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro arm_model:=$m > /dev/null && echo "mujoco $m OK"
done
```

Expected: four `OK` lines.
