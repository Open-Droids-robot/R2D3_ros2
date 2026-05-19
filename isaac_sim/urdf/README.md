# `isaac_sim/urdf/`

Source xacros that compose the V1 R2D3 model for Isaac Sim. The pipeline is **xacro → flat URDF → USD**:

```
       (this dir)                 inside Humble container        inside Isaac Sim container
┌─────────────────────────┐         ┌────────────────────┐         ┌──────────────────────┐
│ r2d3_v1.urdf.xacro      │  xacro  │  r2d3_v1.urdf      │  isaacsim.asset.            │
│ parallel_gripper.xacro  │ ─────▶  │  (flat, rendered)  │   importer.urdf  ─────▶     │ r2d3.usd            │
│                         │         │                    │                              │                     │
└─────────────────────────┘         └────────────────────┘         └──────────────────────┘
                                                                          (scripts/urdf_to_usd.py)
```

## Files

| File | Purpose |
|---|---|
| `r2d3_v1.urdf.xacro` | Composition wrapper: 75b URDF + D435 macro at `head_link2` + parallel gripper on both hands |
| `parallel_gripper.urdf.xacro` | Macro defining a 2-prismatic-joint gripper (one driven, one mimicked) attached to a hand_base_link |
| `render.sh` | Convenience wrapper around `xacro r2d3_v1.urdf.xacro -o r2d3_v1.urdf` (run inside a ROS2 env) |
| `r2d3_v1.urdf` *(generated)* | Flat URDF, consumed by `scripts/urdf_to_usd.py`. Regenerable any time. |

## What this adds on top of the upstream 75b URDF

- **2 parallel grippers** — 4 new joints total (`l_finger_drive`, `l_finger_mimic`, `r_finger_drive`, `r_finger_mimic`), each prismatic with stroke 0–0.035 m. The mimic joints reference the drive joints, so the rendered URDF carries `<mimic joint="l_finger_drive" multiplier="1.0"/>` tags that Isaac Sim's `parse_mimic=True` honors via PhysX Tendons.
- **1 head-mounted D435** — full RealSense kinematic chain (`head_camera_link`, `head_camera_color_frame`, etc.) attached to `head_link2` with a +0.05 m X-forward offset. Uses the existing `sensor_d435` macro from `ros2_realsense2/realsense2_description/`.

Total post-render delta vs upstream 75b: **+4 joints, +4 links** (grippers) and **+N joints/links** from the D435 chain (typically 6–8 frames depending on `use_nominal_extrinsics`).

## How to render

You need `xacro` and the two upstream packages on `ROS_PACKAGE_PATH`. Inside a ROS2 Humble container that has the repo bind-mounted at `/workspace/r2d3_isaac`:

```bash
source /opt/ros/humble/setup.bash
cd /workspace/r2d3_isaac
# Build the description packages so $(find ...) resolves them
colcon build --packages-select dual_rm_75b_description realsense2_description
source install/setup.bash
# Render
bash isaac_sim/urdf/render.sh
```

The output `r2d3_v1.urdf` is gitignored by default (regenerate any time), but you may commit it for review by force-adding.

## Verification

After rendering:

```bash
# parses cleanly
python3 -c 'from lxml import etree; etree.parse("isaac_sim/urdf/r2d3_v1.urdf")'

# gripper joints present
grep -E 'finger_(drive|mimic)' isaac_sim/urdf/r2d3_v1.urdf

# D435 frames present
grep -oE 'name="head_camera[^"]*"' isaac_sim/urdf/r2d3_v1.urdf | sort -u

# Link / joint counts
python3 - <<PY
from lxml import etree
r = etree.parse("isaac_sim/urdf/r2d3_v1.urdf").getroot()
print(f"links:  {len(r.findall('link'))}")
print(f"joints: {len(r.findall('joint'))}")
PY
```

Then feed to `scripts/urdf_to_usd.py` inside the Isaac Sim container.

## Why not just edit the upstream URDF directly?

Two reasons:
1. **Clean branch diff.** Upstream `Docker/docker/...` and `ros2_rm_robot/...` stay untouched on `isaac-sim-v1`; everything Isaac-specific lives under `isaac_sim/`.
2. **Reversible.** When V2 adds dexterous hands or a different gripper, we swap out `parallel_gripper.urdf.xacro` without merging changes back into the 75b package.
