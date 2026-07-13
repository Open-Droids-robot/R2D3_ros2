# R2D3 URDF Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulation robot description faithfully match the real robot by removing the sim-only frame hacks (camera −90°, lidar placeholder + −90° compensation) and consolidating the mesh→nav rotation in one place.

**Architecture:** Approach A from the spec. The core `dual_rm_description` package stays real-faithful (only the camera hack is reverted). The `dual_rm_simulation` overlay keeps the single mesh→nav +90° at `base_footprint_to_base` and mounts sensors in the +X-forward nav frame (`base_footprint`) at the real robot's coordinates, with zero compensation rotations. The lidar frame is renamed `lidar_link` → `laser_link` to match the real robot.

**Tech Stack:** ROS 2 Humble, `xacro`, `check_urdf` (urdfdom), `tf2_ros`, Gazebo (Gz Sim / Harmonic), `diff_drive_controller`.

**Reference spec:** `docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md`

## Global Constraints

- Scope is the **sim repo only** (`sim_ws/src/R2D3_ros2`). Do **not** edit `ros2_ws` or `nav2_j`.
- Do **not** touch inertia, mass, COM, or any joint origin other than those named here — they already match the real robot exactly.
- Keep `joint_left_wheel` axis `1 0 0` (correct ros2_control fix; do not revert).
- Canonical convention: REP-105, `base_footprint` is the +X-forward nav frame.
- The mesh→nav +90° yaw lives in exactly one place: `base_footprint_to_base`. No sensor may carry a compensating rotation.
- Real robot laser reference value (verbatim, real `base_link_underpan` → `laser_link`): `xyz="0.325 0 0.210" rpy="0 0 0"`.
- Work on branch `sim` (already checked out). Commit after each task.

---

### Task 0: Build description packages so xacro/check_urdf resolve

**Files:**
- Modify: none (workspace build only)

- [ ] **Step 1: Build the two packages**

Run:
```bash
cd /home/r2d3/sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_rm_description dual_rm_simulation
```
Expected: `Finished <<< dual_rm_description` and `Finished <<< dual_rm_simulation`, no errors.

- [ ] **Step 2: Source and confirm packages are found**

Run:
```bash
source /home/r2d3/sim_ws/install/setup.bash
ros2 pkg prefix dual_rm_description && ros2 pkg prefix dual_rm_simulation
```
Expected: both print an install path (no "not found").

- [ ] **Step 3: Baseline flatten (captures the current hacks so later diffs are meaningful)**

Run:
```bash
xacro $(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/urdf/r2d3_description.urdf.xacro arm_model:=65b > /tmp/r2d3_desc_before.urdf
grep -A2 'name="camera_joint"' /tmp/r2d3_desc_before.urdf | grep rpy
```
Expected: prints `rpy="0 0 -1.5707963267948966"` (the current hack — confirms baseline).

No commit (build artifacts only).

---

### Task 1: Revert `camera_joint` yaw to 0° in the core description

**Files:**
- Modify: `ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro:171`

**Interfaces:**
- Produces: `camera_link` frame with zero yaw relative to `head_link2` (matches real robot's `camera_joint rpy="0 0 0"`).

- [ ] **Step 1: Write the failing check**

The camera is physically straight; the flattened description must show `camera_joint` rpy = 0. Run this check now (it should FAIL against the current hack):
```bash
cd /home/r2d3/sim_ws && source install/setup.bash
xacro $(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/urdf/r2d3_description.urdf.xacro arm_model:=65b \
  | grep -A2 'name="camera_joint"' | grep -q 'rpy="0 0 0"' && echo PASS || echo FAIL
```
Expected now: `FAIL` (still `rpy="0 0 -pi/2"`).

- [ ] **Step 2: Apply the edit**

In `urdf/body/body_head_platform.urdf.xacro`, change the `camera_joint` origin (line 171):
```xml
      rpy="0 0 ${-pi/2}" />
```
to:
```xml
      rpy="0 0 0" />
```

- [ ] **Step 3: Rebuild and re-run the check**

Run:
```bash
cd /home/r2d3/sim_ws && colcon build --packages-select dual_rm_description >/dev/null && source install/setup.bash
xacro $(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/urdf/r2d3_description.urdf.xacro arm_model:=65b \
  | grep -A2 'name="camera_joint"' | grep -q 'rpy="0 0 0"' && echo PASS || echo FAIL
```
Expected: `PASS`.

- [ ] **Step 4: Validate the tree still parses (both arm variants)**

Run:
```bash
for m in 65b 75b; do
  xacro $(ros2 pkg prefix dual_rm_description)/share/dual_rm_description/urdf/r2d3_description.urdf.xacro arm_model:=$m > /tmp/desc_$m.urdf \
    && check_urdf /tmp/desc_$m.urdf | head -1
done
```
Expected: each prints `robot name is: r2d3_description` with no parse error.

- [ ] **Step 5: Commit**

```bash
cd /home/r2d3/sim_ws/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_description/dual_rm_description/urdf/body/body_head_platform.urdf.xacro
git commit -m "fix(desc): revert camera_joint yaw to 0 (camera is mounted straight)

Removes the sim-only -90deg render hack from the shared description so the
core description matches the real robot (camera_joint rpy 0 0 0). The
sideways-image issue is addressed at the sim sensor/frame layer, not here.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Rename lidar frame to `laser_link` and drop compensation in the macro

**Files:**
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/sensors/lidar.urdf.xacro` (link name, joint child, `gz_frame_id`, any `gazebo reference`)

**Interfaces:**
- Produces: `laser_link` frame published by the Gz lidar sensor (`gz_frame_id=laser_link`), matching the real robot's frame name.
- Consumes: `parent_link`, `xyz`, `rpy` macro params (unchanged signature).

- [ ] **Step 1: Write the failing check**

Run (should FAIL — frame is still `lidar_link`):
```bash
grep -q 'name="laser_link"' /home/r2d3/sim_ws/src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/sensors/lidar.urdf.xacro && echo PASS || echo FAIL
```
Expected now: `FAIL`.

- [ ] **Step 2: Rename all occurrences of `lidar_link` → `laser_link`**

In `urdf/sensors/lidar.urdf.xacro`, replace every `lidar_link` with `laser_link` (the `<link name=...>`, the `<child link=...>` of `lidar_joint`, the `<gz_frame_id>`, and any `<gazebo reference=...>`). Leave the macro name `lidar_sensor` and joint name `lidar_joint` unchanged (those are not TF frames). Also update the macro's default `xyz` to the real value so a bare call is already correct:
```xml
    <xacro:macro name="lidar_sensor" params="parent_link xyz:='0.325 0 0.210' rpy:='0 0 0'">
```

- [ ] **Step 3: Verify the rename is complete**

Run:
```bash
F=/home/r2d3/sim_ws/src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/sensors/lidar.urdf.xacro
grep -c 'lidar_link' "$F"; grep -c 'laser_link' "$F"
```
Expected: first line `0` (no `lidar_link` left), second line `>=3` (`laser_link` present in link, joint child, gz_frame_id).

- [ ] **Step 4: Commit**

```bash
cd /home/r2d3/sim_ws/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_simulation/urdf/sensors/lidar.urdf.xacro
git commit -m "refactor(sim): rename lidar_link -> laser_link, default mount to real value

Aligns the sim lidar frame name with the real robot (laser_link) and sets
the macro default mount to the real 0.325 0 0.210 with no compensation rpy.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Mount lidar in the nav frame at real coords; remove the −90° from the call site

**Files:**
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro:34-35`

**Interfaces:**
- Consumes: `base_footprint` (nav frame, +X forward, defined at `r2d3_sim.urdf.xacro:22`) and the `lidar_sensor` macro (Task 2).
- Produces: `base_footprint` → `laser_link` static transform equal to `xyz=(0.325, 0, 0.210), rpy=0`, identical to the real robot's `base_link_underpan` → `laser_link`.

- [ ] **Step 1: Write the failing check**

A helper that flattens `r2d3_sim` and composes the `base_footprint` → `laser_link` transform, asserting it equals the real value. Create `scripts/check_frame.py`:
```python
#!/usr/bin/env python3
import sys, numpy as np, xml.etree.ElementTree as ET
def R(rpy):
    r,p,y=rpy; cx,sx=np.cos(r),np.sin(r); cy,sy=np.cos(p),np.sin(p); cz,sz=np.cos(y),np.sin(y)
    return (np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
            @ np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
            @ np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]]))
def parse(path):
    root=ET.parse(path).getroot(); J={}
    for j in root.findall('joint'):
        c=j.find('child'); o=j.find('origin')
        if c is None: continue
        xyz=[float(v) for v in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]
        rpy=[float(v) for v in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
        p=j.find('parent')
        J[c.get('link')]=(p.get('link') if p is not None else None, np.array(xyz), R(rpy))
    return J
def chain(J, frame):
    T=np.eye(4)
    while frame in J and J[frame][0] is not None:
        parent,t,Rm=J[frame]
        M=np.eye(4); M[:3,:3]=Rm; M[:3,3]=t
        T=M@T; frame=parent
    return T, frame
path, child = sys.argv[1], sys.argv[2]
J=parse(path); T,root=chain(J, child)
exp_t=np.array([float(v) for v in sys.argv[3].split()])
exp_R=R([float(v) for v in sys.argv[4].split()])
ok = root=='base_footprint' and np.allclose(T[:3,3],exp_t,atol=1e-4) and np.allclose(T[:3,:3],exp_R,atol=1e-4)
print("root=",root," t=",np.round(T[:3,3],4))
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
```
Run (should FAIL — lidar still at `0 -0.24 0` with −90° on `base_link_underpan`):
```bash
cd /home/r2d3/sim_ws && source install/setup.bash
xacro $(ros2 pkg prefix dual_rm_simulation)/share/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b > /tmp/r2d3_sim.urdf
python3 src/R2D3_ros2/scripts/check_frame.py /tmp/r2d3_sim.urdf laser_link "0.325 0 0.210" "0 0 0"
```
Expected now: `FAIL`.

- [ ] **Step 2: Edit the call site**

In `urdf/r2d3_sim.urdf.xacro`, change the lidar call (lines 34-35) from:
```xml
    <xacro:lidar_sensor parent_link="base_link_underpan"
                        xyz="0.0 -0.24 0.0" rpy="0 0 ${-pi/2}"/>
```
to (attach to the nav frame at the real coordinates, no compensation):
```xml
    <xacro:lidar_sensor parent_link="base_footprint"
                        xyz="0.325 0 0.210" rpy="0 0 0"/>
```

- [ ] **Step 3: Rebuild, re-flatten, re-run the check**

Run:
```bash
cd /home/r2d3/sim_ws && colcon build --packages-select dual_rm_simulation >/dev/null && source install/setup.bash
xacro $(ros2 pkg prefix dual_rm_simulation)/share/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b > /tmp/r2d3_sim.urdf
python3 src/R2D3_ros2/scripts/check_frame.py /tmp/r2d3_sim.urdf laser_link "0.325 0 0.210" "0 0 0"
```
Expected: `root= base_footprint` and `PASS`.

- [ ] **Step 4: Confirm no residual −90° compensation and the tree parses**

Run:
```bash
grep -n 'pi/2' /home/r2d3/sim_ws/src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro
check_urdf /tmp/r2d3_sim.urdf | head -1
```
Expected: the only `pi/2` line is the `base_footprint_to_base` joint (line ~27); the lidar call has none. `check_urdf` prints `robot name is: r2d3_sim`.

- [ ] **Step 5: Commit**

```bash
cd /home/r2d3/sim_ws/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro scripts/check_frame.py
git commit -m "fix(sim): mount laser in nav frame at real coords, drop -90 compensation

Attaches laser_link to base_footprint (nav, +X-forward) at the real
0.325 0 0.210 with no compensating yaw. base_footprint -> laser_link now
equals the real base_link_underpan -> laser_link. Adds check_frame.py.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Make the IMU frame nav-consistent

**Files:**
- Modify: `ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro:38-39`

**Interfaces:**
- Consumes: `base_footprint`, `imu_sensor` macro.
- Produces: `imu_link` expressed in the +X-forward nav frame (no 90° yaw error vs the mesh frame).

- [ ] **Step 1: Inspect the current IMU attachment**

Run:
```bash
grep -n -A2 'imu_sensor' /home/r2d3/sim_ws/src/R2D3_ros2/ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro
```
Expected: `parent_link="base_link_underpan" xyz="0 0 0" rpy="0 0 0"` — i.e. IMU currently inherits the mesh frame (+Y forward), a 90° yaw off the nav frame.

- [ ] **Step 2: Re-parent the IMU to the nav frame**

Change the imu call (lines 38-39) from:
```xml
    <xacro:imu_sensor parent_link="base_link_underpan"
                      xyz="0 0 0" rpy="0 0 0"/>
```
to:
```xml
    <xacro:imu_sensor parent_link="base_footprint"
                      xyz="0 0 0.233" rpy="0 0 0"/>
```
(The `0.233` keeps the IMU at the chassis height that `base_footprint_to_base` provided; `base_footprint` is +X-forward so the IMU heading now matches the nav convention.)

- [ ] **Step 3: Rebuild and verify IMU frame orientation is identity vs nav frame**

Run:
```bash
cd /home/r2d3/sim_ws && colcon build --packages-select dual_rm_simulation >/dev/null && source install/setup.bash
xacro $(ros2 pkg prefix dual_rm_simulation)/share/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro arm_model:=65b > /tmp/r2d3_sim.urdf
python3 src/R2D3_ros2/scripts/check_frame.py /tmp/r2d3_sim.urdf imu_link "0 0 0.233" "0 0 0"
```
Expected: `root= base_footprint` and `PASS`.

- [ ] **Step 4: Commit**

```bash
cd /home/r2d3/sim_ws/src/R2D3_ros2
git add ros2_rm_robot/dual_rm_simulation/urdf/r2d3_sim.urdf.xacro
git commit -m "fix(sim): re-parent IMU to nav frame (base_footprint) for heading consistency

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Runtime verification in Gazebo (manual gate)

**Files:**
- Modify: none (verification only)

**Interfaces:**
- Consumes: the full sim launch (`dual_rm_simulation/launch/gz_sim.launch.py`).

- [ ] **Step 1: Launch the sim**

Run:
```bash
cd /home/r2d3/sim_ws && source install/setup.bash
ros2 launch dual_rm_simulation gz_sim.launch.py
```
Expected: Gazebo opens with the robot spawned, `/robot_description` published, `diff_drive_controller` and `joint_state_broadcaster` active (`ros2 control list_controllers` shows them `active`).

- [ ] **Step 2: Verify laser & camera transforms match the real values**

In a second sourced terminal:
```bash
ros2 run tf2_ros tf2_echo base_footprint laser_link
ros2 run tf2_ros tf2_echo base_footprint camera_link
```
Expected: `base_footprint`→`laser_link` translation `[0.325, 0.000, 0.210]`, rotation identity. Camera translation/orientation consistent with the head pose and **no** residual 90° yaw.

- [ ] **Step 3: Verify drive direction**

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.2}}}'
```
Expected: robot drives **forward** (along +X of `base_footprint`), not sideways and not spinning. If it spins or strafes, the wheel-axis / `base_footprint` mapping is wrong — stop and debug before proceeding (use superpowers:systematic-debugging).

- [ ] **Step 4: Verify camera image is upright**

```bash
ros2 run rqt_image_view rqt_image_view /camera/image
```
Expected: image is upright (not rotated 90°) with `camera_joint` at 0°. If it is rotated, the root cause is in the Gz camera sensor pose/optical convention — debug at the sensor layer (do **not** re-add the −90° to the description).

- [ ] **Step 5: Record results**

Append a short "Verification results" note (pass/fail per step, with the tf2_echo numbers) to the spec file `docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md`, then commit:
```bash
cd /home/r2d3/sim_ws/src/R2D3_ros2
git add -f docs/superpowers/specs/2026-07-13-r2d3-urdf-unification-design.md
git commit -m "docs: record sim URDF unification verification results

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Notes on out-of-scope items (do not implement now)

- MJCF regeneration from the unified description (separate plan).
- Package rename / `package://` mesh URIs (later hygiene pass).
- Real-robot repo cleanup (`nav2_r2d3/urdf/urdf/` duplicate, double laser definition).
