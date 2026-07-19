# Wrist RealSense D435 Cameras — Design

Date: 2026-07-19
Status: Approved, not yet implemented

## Goal

Add two RealSense D435 cameras to the R2D3 robot, one on each wrist, publishing
under `/left_wrist/**` and `/right_wrist/**`. The mount angle (pan/tilt) must be
adjustable so each camera can be aimed at its gripper.

Targets: **Gazebo Harmonic sim and MuJoCo sim.** Real-robot bringup is out of
scope for this spec (the real robot does carry D435s on the wrists, so the topic
layout is chosen to make that a later launch-file-only change).

## The mount

The wrist meshes already contain the camera housing — no new CAD is needed. It
is a rectangular bar on every wrist link, mirrored per arm:

| mesh              | lobe X side | center (m)              | size XYZ (m)            |
| ----------------- | ----------- | ----------------------- | ----------------------- |
| 65b `l_link6`     | +X          | `0.0664, 0, 0.0304`     | `0.024 × 0.113 × 0.037` |
| 65b `r_link6`     | −X          | `-0.0664, 0, 0.0334`    | `0.024 × 0.113 × 0.037` |
| 75b `l_link7`     | −X          | `-0.0664, 0, 0.0304`    | `0.024 × 0.113 × 0.037` |
| 75b `r_link7`     | −X          | `-0.0664, 0, 0.0388`    | `0.024 × 0.113 × 0.058` |

A D435 is 90 × 25 × 25 mm; the bar is 113 × 24 × 37 mm. The camera fits inside
it along Y with the 25 mm body matching the 24 mm bar thickness. The bar is a
D435 housing.

The camera is **contained within the bar**, not bolted to its face. The mount
frame therefore sits at the bar centroid, so pan/tilt pivot the camera in place
inside the housing rather than swinging it out through a wall.

Note the X sign is **not** a left/right rule: 65b-left is +X while 65b-right,
75b-left and 75b-right are all −X. This irregularity is stored as explicit data
(see config below), never derived from the side.

## Frame chain

Per side, defined in `dual_rm_description`:

```
<wrist link>                      l_link6 / r_link6 / l_link7 / r_link7
  └─ fixed: mount joint           xyz+rpy from YAML — hardware geometry
     {side}_wrist_camera_mount    housing centroid, nominal outward bore
       └─ fixed: aim joint        rpy = (0, tilt, pan) from YAML — the tunable
          {side}_wrist_camera_link
            ├─ {side}_wrist_camera_color_frame → ..._color_optical_frame
            └─ {side}_wrist_camera_depth_frame → ..._depth_optical_frame
```

Optical frames use the standard `rpy="-pi/2 0 -pi/2"` rotation, matching
`zed2.urdf.xacro`.

**Why two joints instead of one:** the mount joint encodes the bracket geometry
and is never edited; the aim joint is the only thing a user touches. A config
diff then shows only the aim change, and the pivot stays at the housing centroid.

## Configuration

`dual_rm_description/config/wrist_cameras.yaml`:

```yaml
65b:
  left:  {parent: l_link6, xyz: [ 0.0671, 0, 0.0275], rpy: [0, 0, 0],      pan: 0.0, tilt: 0.0}
  right: {parent: r_link6, xyz: [-0.0671, 0, 0.0301], rpy: [0, 0, 3.1416], pan: 0.0, tilt: 0.0}
75b:
  left:  {parent: l_link7, xyz: [-0.0671, 0, 0.0275], rpy: [0, 0, 3.1416], pan: 0.0, tilt: 0.0}
  right: {parent: r_link7, xyz: [-0.0671, 0, 0.0256], rpy: [0, 0, 3.1416], pan: 0.0, tilt: 0.0}
```

- `xyz` / `rpy` — the bracket: housing centroid and nominal outward bore.
- `pan` — rotation about the mount Z (left/right sweep), radians.
- `tilt` — rotation about the mount Y, radians. **Negative = downward**, toward
  the gripper.
- Nominal `pan: 0, tilt: 0` = camera perpendicular to the wrist, boring along
  the housing's outward normal.

Loaded in xacro via `${xacro.load_yaml(...)}`, keyed by the `arm_model` arg.

Both sims read this one file, so a single edit re-aims the camera in Gz and
MuJoCo identically.

## Files

| file                                                          | change                                                |
| ------------------------------------------------------------- | ----------------------------------------------------- |
| `dual_rm_description/urdf/sensors/d435.urdf.xacro`             | **new** — frame-only macro, sibling of `zed2.urdf.xacro` |
| `dual_rm_description/config/wrist_cameras.yaml`                | **new**                                               |
| `dual_rm_description/urdf/r2d3_description.urdf.xacro`         | instantiate both sides                                |
| `dual_rm_simulation/urdf/sensors/wrist_cams_sim.urdf.xacro`    | **new** — Gz `rgbd_camera` blocks                     |
| `dual_rm_simulation/launch/gz_sim.launch.py`                   | bridge + remap to realsense topic names               |
| `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`                    | MJCF cameras                                          |
| `r2d3_mujoco/urdf/ros2_control/mujoco_ros2_control.urdf.xacro` | `<sensor>` tags                                       |

Frames live in the shared description so both sims inherit identical geometry;
each sim adds only its own sensor block. This is the same seam the ZED 2 uses.

The D435 macro is frame-only (no `<gazebo>` tags), so MuJoCo's converter
ignores nothing and needs no special-casing — mirroring the note already in
`r2d3_mujoco.urdf.xacro` about the ZED.

## Sensor aiming rule

**The aim comes from the URDF joint, never from a Gz `<pose>`.** This is the
issue #11 postmortem rule already documented in `zed2_sim.urdf.xacro`
("never orient a sensor with a `<pose>` — only via the frame it is mounted on").
It is what keeps the YAML as the single source of truth across both sims; a
`<pose>` override would silently desync Gz from MuJoCo.

Gz sensors mount on `{side}_wrist_camera_color_frame` with `gz_frame_id` set to
the corresponding optical frame.

## Topics

Real `realsense2_camera` layout, per side:

- `/{left,right}_wrist/color/image_raw`
- `/{left,right}_wrist/color/camera_info`
- `/{left,right}_wrist/depth/image_rect_raw`
- `/{left,right}_wrist/depth/color/points`

Chosen so a future swap to real D435s is a launch-file change with zero consumer
edits — the same reasoning behind the ZED v5 topic-contract commit (b107c79
lineage). The sim ZED layout was deliberately not mirrored: it is sim-shaped,
and these cameras have a real counterpart.

## D435 optics (sim)

- 87° horizontal FOV (`1.5184` rad)
- 848 × 480
- depth clip 0.3 – 3.0 m
- 15 Hz, matching the ZED sim update rate

## Testing

Extend the existing bore-test pattern in
`dual_rm_simulation/test/test_gz_camera_bore.py`:

1. Each wrist camera's optical frame bores along the direction the config
   specifies, for both `65b` and `75b`.
2. A nonzero `tilt` rotates the bore by exactly that amount.

Test 1 is what catches a mirrored-sign mistake that would bury a camera inside
the wrist link — the most likely failure given the irregular X signs.

## Risks — resolved during planning

1. **`xacro.load_yaml` path resolution — RESOLVED, works.** Verified that
   `$(find ...)` resolves inside `xacro.load_yaml` in this xacro version. No
   fallback needed. Note that xacro properties are lazy, so a `load_yaml` of a
   nonexistent file does *not* error until the value is used — a test that only
   defines the property proves nothing.
2. **75b `r_link7` Z discrepancy — RESOLVED, was a sampling artifact.**
   Re-measuring with the outer 4 mm face excluded shows `r_link7` is
   essentially identical to `l_link7`; the extra height came from a separate
   feature on the outermost face that the 24 mm window had swallowed. Corrected
   housing centroids (used in the config above): outer face at `|x| = 0.0783`
   on all four wrists, centroid `|x| = 0.0671`, Z = 0.0275 / 0.0301 / 0.0275 /
   0.0256 for 65b-left, 65b-right, 75b-left, 75b-right.
3. **Install-space staleness — stands.** This repo does not use
   `--symlink-install`; rebuild before trusting any launch or test result, or
   xacro will pull stale installed files.

## Implementation note discovered during planning

Never name a xacro property `e` — `e` is Euler's number in xacro's expression
namespace and gets silently redefined with a warning.
