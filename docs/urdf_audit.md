# URDF Audit — R2D3 V1 Source Assets

> Read-only audit performed 2026-05-19 on `isaac-sim-v1` branch. Identifies the canonical robot URDF for USD conversion and flags work items uncovered along the way.

## Canonical robot description (chosen for V1)

**`ros2_rm_robot/dual_rm_description/dual_rm_75b_description/`**

The project primer specifies **2× RealMan RM75-B arms** (7-DOF each). The repo ships *both* a 65-B (6-DOF) and a 75-B (7-DOF) description package. Use the 75-B package for V1; the 65-B can stay in-tree but is unused.

| | 65-B (unused for V1) | **75-B (V1 target)** |
|---|---|---|
| Description package | `dual_rm_65b_description/` | **`dual_rm_75b_description/`** |
| Mesh count on disk | 38 STL | **42 STL** |
| Already-rendered URDF | `dual_rm_65b_description.urdf` (≈ same shape) | `dual_rm_75b_description.urdf`, 1996 lines, 42 KB |
| 7th-link meshes | — | `l_link7.STL`, `r_link7.STL` present |

## Structure of `dual_rm_75b_description.urdf`

Parsed with `lxml`:

| | Count |
|---|---|
| Links | **34** |
| Joints (total) | **33** |
| ↳ `revolute` | 16 |
| ↳ `continuous` | 10 |
| ↳ `fixed` | 6 |
| ↳ `prismatic` | 1 |

### Joint breakdown

| Category | Count | Names |
|---|---|---|
| Right arm | 7 revolute | `r_joint1..r_joint7` |
| Left arm | 7 revolute | `l_joint1..l_joint7` |
| Head pan | 1 revolute (`head_joint1`) | `platform_base_link → head_link1`, ±1.256 rad (~±72°) |
| Head tilt | 1 revolute (`head_joint2`) | `head_link1 → head_link2`, ±0.419 rad (~±24°) |
| Body lift | 1 prismatic (`platform_joint`) | `body_base_link → platform_base_link`, range 0–1 m |
| Wheels / casters | 10 continuous | `link_left_wheel`, `link_right_wheel`, 4× swivel-wheel pairs |
| Frame mounts | 6 fixed | rigid attachments |

This matches the primer's robot spec (2× 7-DOF + head + body lift + AGV chassis).

## Meshes

- **All 34 mesh references in the URDF resolve** to files on disk. 0 missing. The URDF uses `package://dual_rm_75b_description/meshes/<file>.STL`.
- **8 mesh files are present but unreferenced** by the URDF — likely legacy / alternate geometry. Listed below for awareness; safe to ignore for V1:
  - `bl_Link.STL`, `br_Link.STL`, `fl_Link.STL`, `fr_Link.STL` (probably an older flat-wheel design superseded by `link_left_wheel.STL` etc.)
  - `l_base_link.STL`, `r_base_link.STL` (alternate base; URDF references `l_base_link1.STL` / `r_base_link1.STL`)
  - `l_hand_link.STL`, `r_hand.STL` (alternate hand geometry; URDF only references `l_hand_base_link.STL` / `r_hand_base_link.STL`)

## Camera (Intel RealSense D435)

**`ros2_realsense2/realsense2_description/`**

- Mesh present: `meshes/d435.dae` (Collada)
- Macro: `urdf/_d435.urdf.xacro` exposes `sensor_d435` xacro macro with params `parent`, `name`, `use_nominal_extrinsics`, `add_plug`, `use_mesh`
- **The D435 is NOT included in the main R2D3 URDF.** It lives as a standalone xacro macro to be composed into a parent. For V1 we either:
  - (a) Write a wrapper xacro that includes `dual_rm_75b_description.urdf.xacro` and inserts a `sensor_d435` at the appropriate link (head? wrist? both?), then re-renders the URDF
  - (b) Add the camera directly in the USD scene after conversion (Isaac Sim authoring)

Option (a) is cleaner if the mounting transform is well-known; (b) is simpler if we want to experiment with sensor placement.

## Work items uncovered

1. ⚠️ **No gripper actuators in the URDF, by design.** The hands appear as fixed link bodies only. The real RM driver (`ros2_rm_robot/dual_rm_driver/rm_driver/`) exposes a **controller-level gripper API** — not joint-level:
   - `Set_Gripper_Position(position)` — single integer command for opening
   - `Set_Gripper_Pick(speed, force)` — force-controlled grip
   - `Set_Gripper_Release(speed)` — open
   - `Get_Gripper_State()` — read state
   - ROS2 topics: `rm_driver/set_gripper_position`, `rm_driver/set_gripper_pick`, ...
   - Subscribes to messages: `rm_ros_interfaces::msg::Gripperset`, `Gripperpick`
   
   For Isaac Sim V1 we still need real physical fingers to simulate contact. The path:
   - Author a parallel-gripper sub-assembly (two prismatic finger joints, mimicked so they move symmetrically) and attach to `l_hand_base_link` / `r_hand_base_link`.
   - Expose the same controller-level API (`Set_Gripper_Position`, ...) over ROS2 from the Isaac side, so policies that already work on the real robot work in sim without code changes. Internally that API maps to the simulated prismatic joints.
2. ⚠️ **AGV chassis is in the URDF** (`woosh_agv.urdf.xacro` is included, contributes the 10 continuous wheel joints + base_link_underpan). V1 wants a stationary base. Options:
   - Build a V1-only xacro that omits the `woosh_agv` include
   - Or leave the AGV in but lock the wheel joints / disable physics on them in the Isaac scene
3. ℹ️ **Camera mounting decision needed** — see "Camera" section above.
4. ℹ️ **Two unused descriptions remain** in the tree (`dual_rm_65b_description/`, `dual_rm_moveit_config/dual_rm_65b_moveit_config/`). Harmless but worth noting in the V1 README so contributors don't accidentally edit them.

## What to do next

In order:

1. Decide on the **gripper joint definition** (look at the real-arm driver source under `ros2_rm_robot/dual_rm_driver/` for hints — the actual robot must be commanding fingers somehow).
2. Decide on **V1 base treatment** — strip the AGV vs. lock joints.
3. Decide on **camera mounting** — head, wrist, or both.
4. Pick the **URDF→USD conversion tool**: Isaac Sim's built-in `urdf_importer` extension is the obvious default; `urdf-to-usd` (CLI) is an alternative.
5. Generate the first USD asset and visually inspect in Isaac Sim. Iterate on mesh scale, axes, and link/joint mapping until it matches expectations in RViz.
