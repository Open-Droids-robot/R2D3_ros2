# R2D3 URDF Unification — Design

- **Date:** 2026-07-13
- **Repo scope:** `sim_ws/src/R2D3_ros2` (simulation repo) only
- **Goal:** Make the simulation robot description faithfully match the real
  robot, resolving all known deltas. Do **not** modify the real robot.
- **Downstream (out of scope now):** regenerate MJCF from the unified
  description and validate in MuJoCo.

---

## 1. Background — what the three sources actually are

| Tag | Path | Package | Structure |
|-----|------|---------|-----------|
| **SIM** | `sim_ws/.../dual_rm_description/dual_rm_description/urdf/r2d3_description.urdf.xacro` | `dual_rm_description` | Modular refactor, `arm_model` arg (65b/75b) |
| **REAL** | `ros2_ws/src/nav2_r2d3/urdf/dual_rm_65b_description.urdf.xacro` | `dual_rm_65b_description` | Flat, 65b-only |
| **VENDOR** | `ros2_ws/.../rmc_aida_l_ros2-develop/.../dual_rm_65b_description/urdf/...` | `dual_rm_65b_description` | Upstream original |

Two facts collapse the problem:

- **`nav2_j` is not a third URDF.** `diff -rq` of the URDF trees of
  `nav2_j/src/nav2_r2d3` vs `ros2_ws/src/nav2_r2d3` is empty (byte-identical).
  The only differences between those packages are nav/SLAM config, launch, and
  agent scripts. For URDF purposes **nav2_j ≡ REAL**.
- **REAL is a near-verbatim fork of VENDOR**, adding only a `laser_link`.

So there are effectively **two** descriptions: the SIM refactor and the
vendor-lineage flat one (REAL = VENDOR + laser).

## 2. Delta analysis (evidence)

All three descriptions were flattened with `xacro` (SIM via a stubbed ament
index so `$(find dual_rm_description)` resolves) and compared structurally.

**Identical across SIM/REAL:**
- Mass, center-of-mass, and full inertia tensor for **all 32 shared links**
  (0 deltas).
- Every joint origin **except** the camera.
- All wheel/swivel joint **origins**.

**Real deltas (SIM vs REAL):**

| Delta | SIM | REAL | Root cause / verdict |
|-------|-----|------|----------------------|
| `camera_joint` yaw | `0 0 -π/2` | `0 0 0` | Core description is real-faithful (real robot `camera_joint`=0). The −90° that lived here was **not** a render hack — it compensated the sim overlay's `base_footprint_to_base` +90° mesh→nav yaw. When reverting it here, the equivalent −90° MUST be re-added in the **sim overlay** (see `depth_camera.urdf.xacro` row), or the head-mounted camera bores nav +Y and `/camera/image` looks 90° left. |
| lidar mount | `0 0.24 0` (in sim overlay, `lidar_link`) | `0.325 0 0.210` (`laser_link`) | Sim value is a wrong placeholder. Real value is correct. → adopt real |
| `joint_left_wheel` axis | `1 0 0` | `-1 0 0` | Changed by Nitin in `521e03e` "Refactored Description". Correct for ros2_control `diff_drive_controller` (needs both wheels same axis to drive straight). → **keep** |
| left wrist (`l_joint6`+`l_hand_joint`) rpy | different literals | different literals | Net `l_hand_base_link` orientation identical to 0.02°. Cosmetic. → ignore |
| `laser_link` presence | absent from description | present | Architecture decision (§4) |

## 3. The unifying root cause — a hidden +90° yaw

`dual_rm_simulation/urdf/r2d3_sim.urdf.xacro` mounts the whole robot under a
`base_footprint` with a **+90° yaw**:

```xml
<joint name="base_footprint_to_base" type="fixed">
    <origin xyz="0 0 0.233" rpy="0 0 ${pi/2}"/>
    <parent link="base_footprint"/> <child link="base_link_underpan"/>
</joint>
```

This is why every sensor carries a compensating **−90°**:
- `camera_joint` −90° (the "image to the left" hack),
- lidar called with `rpy="0 0 -π/2"` and the placeholder side-position `0 -0.24 0`.

**Why the +90° exists (half-legitimate):** the drive wheels are separated along
`base_link_underpan` **X** (left `+0.148`, right `−0.148`) and spin about X, so
the mesh-native robot drives along **±Y**. `diff_drive_controller` and Nav2
(REP-105) assume the base frame's **+X is forward**. The +90° maps
mesh-frame → nav-frame. That part is defensible; the mistake was smearing −90°
compensations across sensors *in the shared description* instead of expressing
sensors once, cleanly, in the nav frame.

## 4. Design — chosen approach (A)

**One canonical physical description; the sim overlay holds only
simulation-specific artifacts.**

**Canonical convention:** REP-105. `base_footprint` is the nav frame with **+X
forward**, exactly as the real robot's EKF treats `base_link_underpan`. Physical
sensors are expressed at the **real robot's coordinates in a +X-forward frame,
with zero compensation rotations**. The mesh→nav +90° lives in exactly **one**
place (`base_footprint_to_base`); nothing downstream compensates for it.

**Frame-correspondence (documented, remap at bringup — no mesh-chain renaming):**
sim `base_footprint` ≡ real `base_link_underpan` (both are the +X-forward nav
frame).

### 4a. Core description — `dual_rm_description/` (real-faithful)

| File | Change |
|------|--------|
| `urdf/body/body_head_platform.urdf.xacro` | `camera_joint` rpy `0 0 -π/2` → **`0 0 0`** |
| (package) | No other change. Inertia/geometry already match; left-wheel axis `1 0 0` stays |
| `urdf/legacy/` | Leave as-is; confirm nothing loads it |

The core description carries **no** `base_footprint`, **no** sensor plugins,
**no** compensation rotations.

### 4b. Sim overlay — `dual_rm_simulation/`

| File | Change |
|------|--------|
| `urdf/r2d3_sim.urdf.xacro` | Keep the single `base_footprint_to_base` +90° as the only mesh→nav mapping. Update the `lidar_sensor` call: drop the `−π/2` compensation and the `0 -0.24 0` placeholder; mount the laser at the real value (final numbers pinned empirically, see §5) |
| `urdf/sensors/lidar.urdf.xacro` | Rename `lidar_link` → **`laser_link`**; set `gz_frame_id` to match; default mount = real value; no compensation rpy |
| `urdf/sensors/depth_camera.urdf.xacro` | `camera_optical_joint` rpy `-π/2 0 -π/2` → **`-π/2 0 -π`** — folds the −90° base-yaw compensation into the sim-only optical frame so the camera bores nav +X. |
| `urdf/sensors/imu.urdf.xacro` | Confirm mount; no compensation rpy |

**Deliberate open item — exact sensor xyz/rpy in the overlay.** The sim
conflated a height offset (base_footprint at ground; base_link_underpan +0.233)
*and* the +90° yaw into frames that do not map 1:1 onto the real robot's single
`base_link_underpan`. Exact laser/imu transforms are therefore **not**
hardcoded here; they are pinned during implementation by matching `tf2_echo`
against the real robot (§5). This spec fixes structure and target; the
implementation fixes the last decimals empirically.

## 5. Verification (evidence-gated)

1. `xacro` flattens `r2d3_description` (65b + 75b) and `r2d3_sim` with no error.
2. `check_urdf` on the flattened output: single tree, expected link/joint counts.
3. Gazebo spawn: command `+x` twist → robot **drives forward, not sideways**
   (validates wheel-axis + base_footprint mapping end-to-end).
4. `ros2 run tf2_ros tf2_echo <nav_frame> laser_link` (and camera) in sim equals
   the real-robot values.
5. Camera image **upright and forward** in RViz/rqt: `camera_joint`=0° in the
   core description AND the −90° base-yaw compensation present in the sim
   overlay's `camera_optical_joint` (`-π/2 0 -π`).
6. (Out of scope now) MJCF export consumes the same canonical description.

## 6. Cleanup / cruft noted (not required, but recommended)

- `ros2_ws/src/nav2_r2d3/urdf/urdf/` — nested duplicate of the urdf dir whose
  flattened `.urdf` drifts from the outer one. (Real repo; out of scope.)
- Laser defined twice on the real robot (URDF vs `static_transform_publisher` in
  rtabmap launches). (Real repo; out of scope.)
- Multiple flattened `.urdf` artifacts drift independently — treat flattened
  URDFs as build artifacts, not sources.

## 7. Non-goals

- No changes to the real robot repos (`ros2_ws`, `nav2_j`).
- No package rename or mesh-URI change in this pass (can be a later hygiene pass;
  `package://` is preferred for MJCF portability but not required now).
- No frame renaming on the mesh chain; name differences handled by remap.
