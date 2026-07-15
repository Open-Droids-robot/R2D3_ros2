# Camera Optical-Frame Yaw Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the MuJoCo `/camera/image` look forward (nav +X) instead of 90° to the robot's left, by adding the sim-overlay yaw compensation that the URDF unification removed but never re-added.

**Architecture:** The sim overlay injects a mesh→nav **+90° yaw** at `base_footprint_to_base` (`r2d3_mujoco.urdf.xacro:29`). That yaw propagates down the whole mechanical tree to `camera_link`, so the camera's optical bore (which the MuJoCo converter takes from the `camera_optical_frame` site) points nav **+Y (left)**. The lidar/imu escape the yaw by mounting directly on `base_footprint`; the head-mounted camera cannot, so it needs a **−90° yaw compensation** in its optical frame. We add that compensation in the sim-only, both-sims-shared `depth_camera.urdf.xacro`, leaving the core `dual_rm_description` real-faithful (`camera_joint rpy="0 0 0"`). This *completes* the unification intent — commit `9a01958` correctly removed the sim-hack from the core description but forgot to add the compensation back in the overlay.

**Tech Stack:** ROS 2 Jazzy, xacro, `mujoco_ros2_control` 0.0.3, MuJoCo 3.x, Python `unittest`.

## Global Constraints

- **Scope: MuJoCo only.** Do not attempt to fix Gazebo in this plan. See the "Gazebo caveat" section — the shared-file edit may leave Gz's frame label inconsistent; that is tracked as separate work.
- **Do not modify the core description** `dual_rm_description/.../body_head_platform.urdf.xacro`. `camera_joint` stays `rpy="0 0 0"` (matches the real robot). Do **not** revert commit `9a01958`.
- **Verified fix value:** `camera_optical_joint` origin `rpy="${-pi/2} 0 ${-pi}"` yields optical bore = nav +X, right = nav −Y, down = nav −Z (a correct upright REP-103 optical frame). This was checked numerically against the measured `camera_link` orientation (`camera_link` X = `base_footprint` +Y).
- Frame/rotation convention: URDF fixed-axis rpy, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.

---

## File Structure

- `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro` — **the fix.** One-line rpy change on `camera_optical_joint` + explanatory comment. Sim-only, shared by both sims.
- `r2d3_mujoco/test/test_camera_optical_frame.py` — **new** regression test. Flattens the MuJoCo robot xacro and asserts the `camera_optical_frame` bore points nav +X in `base_footprint`. Would have caught `9a01958`.
- `docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md` — correct the wrong claim that `camera_joint=0` yields an upright image.
- `docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md` — correct the corresponding verification step.

---

### Task 1: Regression test for the camera optical bore

**Files:**
- Test (create): `r2d3_mujoco/test/test_camera_optical_frame.py`

**Interfaces:**
- Consumes: `r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro` (flattened via the `xacro` CLI), which pulls in `depth_camera.urdf.xacro`.
- Produces: nothing consumed by later tasks (a standalone guard).

- [ ] **Step 1: Write the failing test**

Create `r2d3_mujoco/test/test_camera_optical_frame.py`:

```python
"""Regression guard: the camera's optical frame must bore along the robot's
nav-forward direction (base_footprint +X), upright.

The sim overlay yaws the whole mechanical tree +90deg at base_footprint_to_base
to reconcile the mesh frame with the Nav2 convention. The head-mounted camera
rides that yaw, so its optical frame carries a -90deg compensation (in the
sim-only depth_camera.urdf.xacro). If that compensation is dropped -- as it was
in commit 9a01958 -- the camera bores along base_footprint +Y and /camera/image
looks 90deg to the robot's left. This test recomputes the base_footprint ->
camera_optical_frame transform from the flattened URDF and asserts the bore.
"""
import math
import shutil
import subprocess
import unittest
from pathlib import Path
from xml.dom import minidom

import numpy as np

XACRO = Path(__file__).resolve().parent.parent / "urdf" / "r2d3_mujoco.urdf.xacro"


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

    # Walk up from child to root, collecting rotations, then compose top-down.
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


class TestCameraOpticalFrame(unittest.TestCase):
    def test_bore_points_nav_forward(self):
        urdf = _flatten_urdf()
        R = _joint_orientation_chain(urdf, "camera_optical_frame")
        bore = R[:, 2]   # optical +Z = viewing direction
        down = R[:, 1]   # optical +Y = down in image
        np.testing.assert_allclose(bore, [1.0, 0.0, 0.0], atol=1e-6,
                                   err_msg=f"optical bore should be nav +X, got {bore}")
        np.testing.assert_allclose(down, [0.0, 0.0, -1.0], atol=1e-6,
                                   err_msg=f"optical down should be nav -Z (upright), got {down}")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails (against the current bug)**

Run: `cd r2d3_mujoco && python3 -m pytest test/test_camera_optical_frame.py -v`
Expected: FAIL — `optical bore should be nav +X, got [0. 1. 0.]` (current buggy state). If it SKIPs, source the workspace (`source install/setup.bash`) so `xacro` and the `dual_rm_*` packages resolve, then rerun until it FAILs.

- [ ] **Step 3: Commit the failing test**

```bash
git add r2d3_mujoco/test/test_camera_optical_frame.py
git commit -m "test(r2d3_mujoco): guard camera optical bore points nav-forward (currently red)"
```

---

### Task 2: Add the −90° optical-frame compensation

**Files:**
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro:9-16`
- Test: `r2d3_mujoco/test/test_camera_optical_frame.py` (from Task 1)

**Interfaces:**
- Consumes: the Task 1 test.
- Produces: a `camera_optical_frame` whose bore is nav +X.

- [ ] **Step 1: Apply the fix**

In `ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`, replace the optical-frame block (the `camera_optical_frame` link + `camera_optical_joint`) with:

```xml
        <!-- ── Optical frame (Z-forward, X-right, Y-down per REP-103) ── -->
        <link name="camera_optical_frame"/>
        <joint name="camera_optical_joint" type="fixed">
            <parent link="camera_link"/>
            <child link="camera_optical_frame"/>
            <!-- Body-frame (X-fwd) -> optical (Z-fwd) is the standard
                 rpy="-pi/2 0 -pi/2". The trailing yaw carries an EXTRA -pi/2
                 (net "-pi/2 0 -pi") to compensate the sim overlay's mesh->nav
                 +pi/2 yaw at base_footprint_to_base, which yaws the whole
                 mechanical tree -- including this head-mounted camera_link
                 (camera_link X = base_footprint +Y). Without it the optical
                 bore points nav +Y and the MuJoCo /camera/image looks 90 deg
                 to the robot's left. The core description's camera_joint stays
                 0 (real-faithful); this compensation is sim-only. Do NOT
                 "simplify" back to -pi/2 -- see commit 9a01958 and
                 docs/superpowers/plans/2026-07-15-camera-optical-frame-yaw-fix.md. -->
            <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi}"/>
        </joint>
```

- [ ] **Step 2: Run the test to verify it passes**

Run: `cd r2d3_mujoco && python3 -m pytest test/test_camera_optical_frame.py -v`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add ros2_rm_robot/dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro
git commit -m "fix(sim): compensate base +90 yaw in camera optical frame (camera bores forward)

The URDF unification (9a01958) removed the camera_joint -90 hack from the
core description (correct: real robot camera_joint=0) but never re-added the
compensation in the sim overlay, so the head-mounted camera rode the
base_footprint_to_base +90 mesh->nav yaw and /camera/image looked 90 deg
left. Fold the -90 into the sim-only camera_optical_joint. MuJoCo only;
Gazebo tracked separately."
```

---

### Task 3: Verify the live MuJoCo camera renders forward + upright

**Files:** none (verification only).

- [ ] **Step 1: Force-regenerate the MJCF (the URDF change invalidates the cache)**

Run:
```bash
rm -rf ~/.ros/r2d3_mujoco/65b
```
The next launch reconverts automatically (checksum changed). Alternatively pass `force_recompile:=true` in Step 2.

- [ ] **Step 2: Launch and eyeball the image (realistic verification)**

Run:
```bash
ros2 launch r2d3_mujoco mujoco_sim.launch.py force_recompile:=true
# in another sourced terminal:
ros2 run rqt_image_view rqt_image_view /camera/image
```
Expected: the image is **upright** (sky up, floor down) AND looks **forward** — you see what is in front of the robot's drive direction, not the scene 90° to its left. Before the fix the same view showed the left side.

- [ ] **Step 3: (Optional) headless bore check against the regenerated MJCF**

If a MuJoCo Python install with an offscreen GL backend is available, confirm the fixed camera's world bore. Run (adjust the interpreter/`MUJOCO_GL` to your MuJoCo env):

```bash
MUJOCO_GL=egl python3 - <<'PY'
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path("/root/.ros/r2d3_mujoco/65b/mujoco_description_formatted.xml".replace("/root", __import__("os").path.expanduser("~")))
d = mujoco.MjData(m); mujoco.mj_forward(m, d)
cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "camera")
R = d.cam_xmat[cid].reshape(3, 3)
print("look(-Z):", np.round(-R[:, 2], 2), " up(+Y):", np.round(R[:, 1], 2))
assert np.allclose(-R[:, 2], [1, 0, 0], atol=1e-2), "camera should look nav +X"
assert np.allclose(R[:, 1], [0, 0, 1], atol=1e-2), "camera up should be world +Z"
print("OK: camera bores nav +X, upright")
PY
```
Expected: `OK: camera bores nav +X, upright`.

- [ ] **Step 4: Confirm the pointcloud frame is consistent (no regression)**

Run (sim still up):
```bash
ros2 topic echo --once /camera/image --field header.frame_id
ros2 run tf2_ros tf2_echo base_footprint camera_optical_frame
```
Expected: `frame_id` is `camera_optical_frame`; the tf2 rotation matches an optical frame whose +Z (bore) points along `base_footprint` +X. No error output.

---

### Task 4: Correct the unification spec + plan docs

**Files:**
- Modify: `docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md`
- Modify: `docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md`

**Interfaces:** documentation only.

- [ ] **Step 1: Fix the spec's camera claim**

In `docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md`, find the camera row in the frame-changes table (the one describing `camera_joint` yaw `0 0 -π/2` → `0 0 0` as a "Sim-render hack ... to mask a sideways image") and the `depth_camera.urdf.xacro` row that says it is **Unchanged** because "the optical frame `(-π/2,0,-π/2)` is already correct." Replace them so they read (adjust surrounding table formatting to match):

```markdown
| `camera_joint` yaw | `0 0 -π/2` | `0 0 0` | Core description is real-faithful (real robot `camera_joint`=0). The −90° that lived here was **not** a render hack — it compensated the sim overlay's `base_footprint_to_base` +90° mesh→nav yaw. When reverting it here, the equivalent −90° MUST be re-added in the **sim overlay** (see `depth_camera.urdf.xacro` row), or the head-mounted camera bores nav +Y and `/camera/image` looks 90° left. |
```

```markdown
| `urdf/sensors/depth_camera.urdf.xacro` | `camera_optical_joint` rpy `-π/2 0 -π/2` → **`-π/2 0 -π`** — folds the −90° base-yaw compensation into the sim-only optical frame so the camera bores nav +X. |
```

- [ ] **Step 2: Fix the spec's verification claim**

In the same file, find the acceptance line stating the camera image is **upright** "with `camera_joint`=0° (no hack)" and replace it with:

```markdown
5. Camera image **upright and forward** in RViz/rqt: `camera_joint`=0° in the
   core description AND the −90° base-yaw compensation present in the sim
   overlay's `camera_optical_joint` (`-π/2 0 -π`).
```

- [ ] **Step 3: Fix the plan's camera verification step**

In `docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md`, find "Step 4: Verify camera image is upright" and replace its Expected note (the one saying "If it is rotated, the root cause is in the Gz camera sensor ... do not re-add the −90° to the description") with:

```markdown
Expected: image is upright AND forward-facing (you see the robot's drive
direction, not the scene 90° to its left). This requires the sim-overlay
compensation `camera_optical_joint rpy="-π/2 0 -π"` in
`dual_rm_simulation/urdf/sensors/depth_camera.urdf.xacro`. Do **not** re-add
the −90° to the core description (`camera_joint` stays 0). If MuJoCo is fixed
but Gazebo is still off, that is the known Gz sensor-convention gap — track
separately, do not change the core description.
```

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md \
        docs/superpowers/plans/2026-07-13-r2d3-urdf-unification.md
git commit -m "docs: correct unification camera claim (needs sim-overlay -90 compensation)"
```

---

## Gazebo caveat (out of scope — read before executing)

The fix edits the **shared** `depth_camera.urdf.xacro`, but the scope is **MuJoCo only**. The two sims derive the camera bore differently:

- **MuJoCo** attaches its `<camera>` to the `camera_optical_frame` *site* (`mujoco_inputs.urdf.xacro:181`), so reorienting that frame fixes the render. ✅ handled here.
- **Gazebo**'s `rgbd_camera` sensor is referenced to `camera_link` and uses `gz_frame_id` to *label* its output. Its actual render direction likely follows `camera_link` (unverified), which this edit does **not** change — so after this fix Gz may render nav +Y while labeling the frame nav +X (inconsistent), or may already handle optical framing itself.

Do not try to fix Gz in this plan. After Task 3, if you have a Gz sim available, sanity-check `/camera/image` there; if it is wrong, open a separate task to fix Gz at the sensor layer (e.g. de-yaw `camera_link` in the Gz overlay or set a sensor pose offset). The current `2026-07-13` plan already flags Gz camera as separate sensor-layer work.

---

## Self-Review

- **Spec coverage:** Root cause (base +90° yaw uncompensated on the head camera) → Task 2 adds the −90°. Regression protection → Task 1. Live confirmation → Task 3. Doc correction (the reasoning error that caused the regression) → Task 4. Gz explicitly scoped out.
- **Placeholder scan:** none — exact rpy value, exact file, complete test code, exact commands with expected output.
- **Type consistency:** the test helper names (`_rpy_to_R`, `_joint_orientation_chain`, `_flatten_urdf`) are used consistently; the asserted value `[1,0,0]` matches the verified bore in Global Constraints and Task 2's comment.
