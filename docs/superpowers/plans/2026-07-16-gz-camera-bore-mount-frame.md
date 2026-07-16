# Gz Camera Bore Mount-Frame Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Gazebo camera render nav-forward (issue #11) by mounting the Gz sensor on a sim-only −90°-yawed frame, with a regression test and human-verified live bringup.

**Architecture:** A Gz camera renders along the +X of the frame its `<sensor>` is mounted on; `gz_frame_id` only labels output. The sim overlay yaws the whole tree +90° at `base_footprint_to_base`, so `camera_link` +X = nav +Y. Fix: add massless `camera_gz_frame` (child of `camera_link`, `rpy="0 0 -π/2"`) and re-reference the existing `<gazebo>` sensor block to it. sdformat lumping composes this into the SDF sensor pose (statically verified: `… 0 0 -1.5708` on `head_link2`).

**Tech Stack:** xacro / URDF, sdformat (`gz sdf`), pytest + numpy, colcon, `gh` CLI.

**Spec:** `docs/superpowers/specs/2026-07-16-gz-camera-bore-mount-frame-design.md`

## Global Constraints

- Do **NOT** add a `<pose>` element inside the Gz `<sensor>` block (postmortem constraint — that approach is banned).
- Do **NOT** change mesh geometry or any mechanical joint angle/origin; core description `camera_joint` stays `rpy="0 0 0"`.
- Do **NOT** revert `9a01958` or touch the MuJoCo fix `c7bbfec` (`camera_optical_joint` stays `rpy="${-pi/2} 0 ${-pi}"`).
- Workspace root is `/home/samzpc/code/r2d3`; repo is at `src/R2D3_ros2` inside it. Source `install/setup.bash` before xacro/pytest commands.
- Install space is **copy-not-symlink**: after editing any `.urdf.xacro`, run `colcon build --packages-select dual_rm_simulation` or tests/launches silently use the OLD file.
- `docs/` is gitignored; commit doc files with `git add -f`.
- Live Gazebo verification is done by the human operator (they eyeball `/camera/image`). Do NOT write ad-hoc Gazebo spawn scripts.

---

### Task 1: Regression test + mount-frame fix (TDD)

**Files:**
- Create: `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py`
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro:28-34`

**Interfaces:**
- Consumes: existing `camera_link`, `camera_optical_frame` links in the flattened sim URDF; `r2d3_sim.urdf.xacro` (xacro entry point, arg `arm_model:=65b`).
- Produces: URDF link `camera_gz_frame` + fixed joint `camera_gz_joint` (parent `camera_link`, `rpy="0 0 ${-pi/2}"`); the Gz `<sensor name="depth_camera">` referenced to `camera_gz_frame`. Task 2's doc text and Task 3's live check rely on exactly these names.

- [ ] **Step 1: Write the failing regression test**

Create `ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py`:

```python
"""Regression guard: the Gazebo (Gz Sim) camera must render along the robot's
nav-forward direction (base_footprint +X), upright, and agree with the
``camera_optical_frame`` it labels its output with.

A Gz camera renders along the +X of the frame its ``<sensor>`` is mounted on
(+Z up, +Y left -- the SDF camera body convention). ``gz_frame_id`` only
LABELS the published output; it does NOT reorient the render. The sim overlay
yaws the whole mechanical tree +90deg about Z at ``base_footprint_to_base``
(mesh -> Nav2), so ``camera_link`` +X = base_footprint +Y and an unmounted-
compensated camera images 90deg to the robot's left (issue #11).

The fix mounts the sensor on the sim-only ``camera_gz_frame`` -- a massless
fixed link yawed -90deg off ``camera_link`` -- so the render bores nav +X.
This is the Gz analog of the MuJoCo fix (commit c7bbfec), which reoriented
the frame MuJoCo's camera hangs on. Per the issue #11 postmortem the Gz
``<sensor>`` ``<pose>`` must NOT be used for this compensation.

This test recomputes base_footprint -> sensor-mount orientation from the
flattened sim URDF (joint chain at q=0, composed with any direct ``<pose>``
on the sensor, so the guard holds regardless of mechanism) and asserts the
render bore and the render/label consistency.

NOTE: the xacro include resolves via $(find dual_rm_simulation) -> the
INSTALL space. Rebuild (colcon build --packages-select dual_rm_simulation)
after editing the xacro, or this test sees the old file.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_sim.urdf.xacro"


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


def _joint_orientation_chain(urdf_str, child, root="base_footprint"):
    """Compose the fixed/neutral-config rotation from root down to `child`.

    Revolute/prismatic joints are evaluated at q=0, so only their <origin> rpy
    contributes to orientation. Returns R (child axes expressed in root frame).
    """
    dom = minidom.parseString(urdf_str)
    joints = {}  # child_link -> (parent_link, R_origin)
    for j in dom.getElementsByTagName("joint"):
        p = j.getElementsByTagName("parent")
        c = j.getElementsByTagName("child")
        if not p or not c:
            continue
        o = j.getElementsByTagName("origin")
        rpy = (0.0, 0.0, 0.0)
        if o and o[0].getAttribute("rpy").strip():
            rpy = tuple(float(v) for v in o[0].getAttribute("rpy").split())
        joints[c[0].getAttribute("link")] = (p[0].getAttribute("link"), _rpy_to_R(*rpy))

    chain = []
    node = child
    while node != root:
        if node not in joints:
            raise AssertionError(f"no joint chain from {root} to {child} (stuck at {node})")
        parent, R = joints[node]
        chain.append(R)
        node = parent
    R_total = np.eye(3)
    for R in reversed(chain):  # root -> ... -> child
        R_total = R_total @ R
    return R_total


def _gz_camera_sensor(urdf_str):
    """Return (reference_link, R_pose) for the Gz camera <sensor>.

    reference_link is the link named by the enclosing <gazebo reference=...>.
    R_pose is the rotation of an optional direct-child <pose> ("x y z r p y",
    identity if absent) so the bore assertion holds no matter which mechanism
    orients the render.
    """
    dom = minidom.parseString(urdf_str)
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
            return gz.getAttribute("reference"), _rpy_to_R(*rpy)
    raise AssertionError("no camera <sensor> found in any <gazebo> block")


# Gz builds the published optical frame from the sensor body frame by the fixed
# SDF-camera mapping: optical +Z = sensor +X (forward), optical +X = -sensor +Y
# (right), optical +Y = -sensor +Z (down). Columns are (optX, optY, optZ) in
# sensor-body coords.
_SENSOR_TO_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])


class TestGzCameraBore(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = _flatten_urdf()
        ref, R_pose = _gz_camera_sensor(cls.urdf)
        cls.sensor_ref = ref
        cls.R_sensor = _joint_orientation_chain(cls.urdf, ref) @ R_pose

    def test_sensor_mounted_on_gz_frame(self):
        """Issue #11 postmortem: compensation must be a mount frame, not a
        sensor <pose>. Guards against silently moving it back."""
        self.assertEqual(self.sensor_ref, "camera_gz_frame")

    def test_render_bore_points_nav_forward(self):
        """The Gz render direction must be nav-forward and upright."""
        bore = self.R_sensor[:, 0]  # sensor +X = Gz viewing direction
        up = self.R_sensor[:, 2]    # sensor +Z = up in image
        np.testing.assert_allclose(bore, [1.0, 0.0, 0.0], atol=1e-6,
                                   err_msg=f"Gz render bore should be nav +X, got {bore}")
        np.testing.assert_allclose(up, [0.0, 0.0, 1.0], atol=1e-6,
                                   err_msg=f"Gz image up should be nav +Z (upright), got {up}")

    def test_render_matches_optical_frame_label(self):
        """The Gz render must agree with the frame it is LABELLED with.

        Gz stamps its output ``gz_frame_id=camera_optical_frame`` but derives
        the actual optical axes from the sensor body frame. If those disagree
        with the URDF's ``camera_optical_frame`` TF, consumers place the cloud
        wrong -- the "potentially worse (inconsistent)" failure issue #11
        flagged.
        """
        R_optical_render = self.R_sensor @ _SENSOR_TO_OPTICAL
        R_optical_label = _joint_orientation_chain(self.urdf, "camera_optical_frame")
        np.testing.assert_allclose(
            R_optical_render, R_optical_label, atol=1e-6,
            err_msg="Gz-rendered optical axes disagree with the camera_optical_frame "
                    f"label:\nrender=\n{R_optical_render}\nlabel=\n{R_optical_label}")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/samzpc/code/r2d3 && source /opt/ros/*/setup.bash && source install/setup.bash && \
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```

Expected: `test_sensor_mounted_on_gz_frame` FAILS with `'camera_link' != 'camera_gz_frame'`; `test_render_bore_points_nav_forward` FAILS (bore is `[0, 1, 0]` = nav +Y, the 90°-left bug); `test_render_matches_optical_frame_label` FAILS (render/label mismatch). If everything SKIPS, the workspace isn't sourced/built — fix that first, do not proceed on skips.

- [ ] **Step 3: Implement the mount frame in the sensor xacro**

In `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`, insert the mount frame between the optical-joint block and the `<!-- ── Gazebo physics … -->` comment, and change the sensor's `<gazebo>` reference. The region from the end of `camera_optical_joint` through the sensor `<gazebo>` open tag becomes:

```xml
        <!-- ── Sim-only Gz mount frame ─────────────────────────────── -->
        <!-- A Gz camera renders along the +X of the frame its <sensor>
             is mounted on (gz_frame_id below only LABELS the output).
             camera_link rides the sim overlay's base_footprint_to_base
             +pi/2 mesh->nav yaw, so camera_link +X = base_footprint +Y
             and an uncompensated Gz camera images 90 deg to the robot's
             left. This massless frame de-yaws the mount so the render
             bores nav +X, matching the camera_optical_frame label above.
             Frame-level only: no mesh, no mechanical angles, and per the
             issue #11 postmortem do NOT do this with a <pose> inside the
             <sensor>. Guarded by test/test_gz_camera_bore.py. -->
        <link name="camera_gz_frame"/>
        <joint name="camera_gz_joint" type="fixed">
            <parent link="camera_link"/>
            <child link="camera_gz_frame"/>
            <origin xyz="0 0 0" rpy="0 0 ${-pi/2}"/>
        </joint>

        <!-- ── Gazebo physics ──────────────────────────────────────── -->
        <gazebo reference="camera_link">
            <self_collide>false</self_collide>
        </gazebo>

        <!-- ── Gz Sim RGBD Camera sensor ───────────────────────────── -->
        <gazebo reference="camera_gz_frame">
```

Everything inside the `<sensor>` block stays byte-for-byte unchanged (no `<pose>` added).

- [ ] **Step 4: Rebuild the package (copy-not-symlink install trap)**

```bash
cd /home/samzpc/code/r2d3 && colcon build --packages-select dual_rm_simulation
```

Expected: `Summary: 1 package finished`. Confirm the install file updated:

```bash
grep -c camera_gz_frame install/dual_rm_simulation/share/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro
```

Expected: `3` (link, joint child, gazebo reference).

- [ ] **Step 5: Run the test to verify it passes**

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash && \
python3 -m pytest src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py -v
```

Expected: 3 passed.

- [ ] **Step 6: Static SDF conversion check**

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash && \
xacro src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b > /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/38e33bd1-3c47-4024-aa1c-9c90fe5c406b/scratchpad/fix.urdf && \
gz sdf -p /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/38e33bd1-3c47-4024-aa1c-9c90fe5c406b/scratchpad/fix.urdf > /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/38e33bd1-3c47-4024-aa1c-9c90fe5c406b/scratchpad/fix.sdf && \
grep -A2 "sensor name='depth_camera'" /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/38e33bd1-3c47-4024-aa1c-9c90fe5c406b/scratchpad/fix.sdf | head -5; \
grep "0 0 -1.5708" /tmp/claude-1000/-home-samzpc-code-r2d3-src-R2D3-ros2/38e33bd1-3c47-4024-aa1c-9c90fe5c406b/scratchpad/fix.sdf
```

Expected: conversion succeeds; the sensor block contains `<pose>-0.0032391 -0.051866… 0.061606… 0 0 -1.5708</pose>` (lump offset + the −90° yaw, composed by sdformat).

- [ ] **Step 7: MuJoCo cross-check (shared macro)**

The macro is also included by `r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro`; MuJoCo must still flatten and its optical-frame test must still pass:

```bash
cd /home/samzpc/code/r2d3 && source install/setup.bash && \
xacro src/R2D3_ros2/r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro > /dev/null && echo "mujoco xacro OK" && \
python3 -m pytest src/R2D3_ros2/r2d3_mujoco/test/test_camera_optical_frame.py -v
```

Expected: `mujoco xacro OK`, all MuJoCo optical-frame tests pass. (If that xacro needs an arg like `arm_model:=65b`, add it — check the file's `<xacro:arg>` declarations.)

- [ ] **Step 8: Commit**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2 && \
git add ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro ros2_rm_robot/dual_rm_simulation/test/test_gz_camera_bore.py && \
git commit -m "fix(sim): mount Gz camera on -90deg camera_gz_frame so it renders nav-forward

A Gz camera renders along its mount frame's +X; camera_link rides the
sim overlay's +90deg mesh->nav base yaw, so the image looked 90deg left
and disagreed with its camera_optical_frame label (issue #11). Mount the
sensor on a sim-only massless frame instead of a sensor <pose> (banned
by the issue #11 postmortem). Guarded by test_gz_camera_bore.py."
```

---

### Task 2: Reconcile the planning docs

**Files:**
- Modify: `docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md:369-377` (Step 4 camera-check text)
- Modify: `docs/superpowers/plans/2026-07-15-camera-optical-frame-yaw-fix.md:313-322` ("Gazebo caveat" section)

**Interfaces:**
- Consumes: Task 1's `camera_gz_frame` name and test path `dual_rm_simulation/test/test_gz_camera_bore.py`.
- Produces: docs stating the Gz gap is fixed via the mount frame (referenced by the issue #11 close-out comment in Task 3).

- [ ] **Step 1: Update the unification plan camera expectation**

In `docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md`, replace this paragraph (after the `rqt_image_view` command in Step 4):

```markdown
Expected: image is upright AND forward-facing (you see the robot's drive
direction, not the scene 90° to its left). This requires the sim-overlay
compensation `camera_optical_joint rpy="-π/2 0 -π"` in
`dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`. Do **not** re-add
the −90° to the core description (`camera_joint` stays 0). If MuJoCo is fixed
but Gazebo is still off, that is the known Gz sensor-convention gap — track
separately, do not change the core description.
```

with:

```markdown
Expected: image is upright AND forward-facing (you see the robot's drive
direction, not the scene 90° to its left). Do **not** re-add the −90° to the
core description (`camera_joint` stays 0); both sims are compensated in the
sim-only `dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`:
- **MuJoCo** takes its bore from the `camera_optical_frame` site → fixed via
  `camera_optical_joint rpy="-π/2 0 -π"` (commit `c7bbfec`).
- **Gazebo** renders along its sensor mounting frame's +X (`gz_frame_id` only
  labels output) → fixed by mounting the sensor on the sim-only
  `camera_gz_frame`, a massless frame yawed −90° off `camera_link` (issue
  #11; a `<pose>` inside the `<sensor>` is banned by the issue #11
  postmortem). Guarded by `dual_rm_simulation/test/test_gz_camera_bore.py`.
```

(Exact original wording may differ slightly — locate the paragraph under the `rqt_image_view` command in Step 4 and replace the whole paragraph.)

- [ ] **Step 2: Update the yaw-fix plan's Gazebo caveat**

In `docs/superpowers/plans/2026-07-15-camera-optical-frame-yaw-fix.md`, retitle the section `## Gazebo caveat (out of scope — read before executing)` to `## Gazebo caveat — RESOLVED (issue #11, mount-frame fix)` and replace the section body's final paragraph (the one beginning "Do not try to fix Gz in this plan.") with:

```markdown
**Gz fix (issue #11):** the sensor is mounted on the sim-only
`camera_gz_frame` — a massless fixed frame yawed −90° off `camera_link` — so
the render bores nav +X and the Gz-derived optical cloud axes match the URDF
`camera_optical_frame`. sdformat's fixed-joint lumping composes the yaw into
the SDF sensor pose on `head_link2`. Gz-only (MuJoCo ignores `<gazebo>`
blocks; the extra massless link is inert there). Note: an earlier attempt
that put the yaw in a `<pose>` inside the `<sensor>` was reverted after a
failed bringup verification and is banned per the issue #11 postmortem — the
mount frame achieves the same composed SDF without it. Guarded by
`dual_rm_simulation/test/test_gz_camera_bore.py`.
```

Also update the mid-section bullet `- **Gazebo**'s `rgbd_camera` sensor is referenced to `camera_link` …` to past tense reality:

```markdown
- **Gazebo**'s `rgbd_camera` renders along its **mounting-frame +X**;
  `gz_frame_id` only *labels* the output. After the MuJoCo fix it rendered
  nav +Y (90° left) while labeling the frame nav-forward — the inconsistency
  this caveat warned about. Fixed by re-mounting the sensor (see below).
```

- [ ] **Step 3: Commit (docs are gitignored — force-add)**

```bash
cd /home/samzpc/code/r2d3/src/R2D3_ros2 && \
git add -f docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md docs/superpowers/plans/2026-07-15-camera-optical-frame-yaw-fix.md && \
git commit -m "docs: mark Gz camera bore gap resolved via camera_gz_frame mount (issue #11)"
```

---

### Task 3: Live verification (human-in-the-loop) and issue close-out

**Files:** none (verification + `gh` only).

**Interfaces:**
- Consumes: Task 1's built workspace; the operator's eyes on `/camera/image`.
- Produces: issue #11 resolution comment + close.

- [ ] **Step 1: Ensure a clean machine (the trap that sank attempt #1)**

```bash
pgrep -af "gz sim|ruby.*gz" && echo "STALE GZ — kill before proceeding" || echo "CLEAN"
```

Expected: `CLEAN`. If stale processes exist, kill them (`pkill -9 -f 'gz sim'; pkill -9 -f 'ruby.*gz'`) and re-check until `CLEAN`. Do NOT launch anything while a foreign gz server is alive — the spawner auto-discovers its worlds and hangs (this is exactly what produced the false "fix broke spawning" verdict on 2026-07-15, log `create_268766`: 3.4 h waiting on stale world `w`).

- [ ] **Step 2: Hand the live check to the operator**

Ask the operator to run, in their own terminal:

```bash
ros2 launch r2d3_bringup bringup_sim.launch.py
# and in a second terminal:
ros2 run rqt_image_view rqt_image_view /camera/image
```

and confirm all three: (1) the robot spawns **exactly once** in Gazebo, (2) `/camera/image` looks **forward** (drive direction, not 90° left), (3) Nav2/SLAM comes up as before. Do not write ad-hoc Gazebo scripts to check this yourself — the operator verifies.

- [ ] **Step 3: On operator confirmation, comment and close issue #11**

```bash
gh issue comment 11 --repo Open-Droids-robot/R2D3_ros2 --body "Fixed via sim-only mount frame: the Gz <sensor> is now referenced to \`camera_gz_frame\` (massless, \`rpy=\"0 0 -pi/2\"\` off \`camera_link\`) in \`dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro\`, so the render bores nav +X and matches the \`camera_optical_frame\` label. No sensor <pose> (per postmortem), no mesh/mechanical changes, \`camera_joint\` stays 0. Guarded by \`dual_rm_simulation/test/test_gz_camera_bore.py\`. Verified live by operator: single spawn, /camera/image forward, Nav2 unaffected. Root cause of the reverted first attempt's spawn failure was a stale gz server (log \`create_268766\`: spawner hung 3.4h on foreign world 'w') — clean-machine check is now step 1 of the verification protocol. Design: docs/superpowers/specs/2026-07-16-gz-camera-bore-mount-frame-design.md"
gh issue close 11 --repo Open-Droids-robot/R2D3_ros2
```

Expected: comment posted, issue closed. If the operator reports a problem instead, do NOT close — return to systematic debugging with their observation.

---

## Self-Review

- **Spec coverage:** fix (Task 1 Steps 3–4), regression test (Task 1 Steps 1–2, 5), static SDF proof (Step 6), MuJoCo guard (Step 7), mandatory live protocol incl. clean-machine + rebuild traps (Task 1 Step 4, Task 3 Steps 1–2), docs reconciliation (Task 2), out-of-scope items untouched. ✓
- **Placeholders:** none; all code/commands are complete. ✓
- **Name consistency:** `camera_gz_frame` / `camera_gz_joint` / `test_gz_camera_bore.py` used identically across tasks. ✓
