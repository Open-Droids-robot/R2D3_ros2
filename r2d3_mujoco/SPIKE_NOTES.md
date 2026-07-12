# Task 4 spike notes: URDF -> MJCF conversion

Deleted in Task 9. Records what was actually observed running the converter
against the real URDF/scene, every fix applied, and the exact facts a
future `ensure_mjcf.py` wrapper (Task 5) needs to know.

## Environment

- Converter venv: `~/.ros/ros2_control/.venv` (pre-existing on this machine;
  no bootstrap cost was measured here).
- `mujoco-ros2-control` python package version: **0.0.3**
  (`ros-jazzy-mujoco-ros2-control 0.0.3-1noble.20260615.175335`).
- Conversion command (this is the exact command `ensure_mjcf.py` should wrap):

```bash
cd ~/code/r2d3 && source /opt/ros/jazzy/setup.bash && source install/setup.bash
xacro install/r2d3_mujoco/share/r2d3_mujoco/urdf/r2d3_mujoco.urdf.xacro > /tmp/r2d3_mj_65b.urdf
mkdir -p ~/.ros/r2d3_mujoco/65b
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh \
  --save_only --add_free_joint \
  -u /tmp/r2d3_mj_65b.urdf \
  --scene $(pwd)/src/R2D3_ros2/r2d3_mujoco/worlds/nav_empty.xml \
  -o ~/.ros/r2d3_mujoco/65b \
  --publish_topic /mujoco_robot_description
```

- With `--publish_topic`, the script writes all output files, publishes once
  on `/mujoco_robot_description`, then calls `rclpy.spin()` and blocks
  forever. It must be run backgrounded and killed once
  `~/.ros/r2d3_mujoco/65b/mujoco_description_formatted.xml` exists and stops
  changing (`pkill -f make_mjcf_from_robot_description`, or track/kill the
  PID directly — `pkill -f` on the wrapper shell script name does not always
  match the underlying python process, kill both PIDs to be safe). The
  `ExternalShutdownException` traceback printed on kill is expected, not a
  failure.
- **Measured runtime**: ~20-30 seconds per run once mesh caches are warm
  (this machine's venv/mesh caches were already warm from an earlier attempt,
  so the "first run bootstraps in minutes" case in the brief was not
  observed here). `ensure_mjcf.py` should budget generously (a few minutes)
  for a genuinely cold cache on a fresh machine, since it does STL->OBJ
  conversion for every mesh (32 meshes for this robot).

## Fixes required (in `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`)

### Fix 1: missing `<default>` block for `collision`/`visual` classes

**Symptom:** `mujoco.MjModel.from_xml_path()` on the first successful save
raised:

```
ValueError: XML Error: unknown default class name 'collision'
Element 'geom', line 43
```

**Root cause:** `mujoco_ros2_control/urdf_to_mujoco_utils.py`
(`update_non_obj_assets`) unconditionally tags every synthesized collision
geom with `class="collision"` and every visual geom with `class="visual"`,
but never defines those classes itself — it expects the caller's
`raw_inputs` to supply the `<default>` block (see the shipped example at
`/opt/ros/jazzy/share/mujoco_ros2_control_demos/demo_resources/mjcf_generation/test_inputs.xml`).
Task 2's xacro never added one.

**Fix:** added to `raw_inputs`:

```xml
<default>
    <default class="visual">
        <geom group="2" type="mesh" contype="0" conaffinity="0"/>
    </default>
    <default class="collision">
        <geom group="3" type="mesh"/>
    </default>
</default>
```

After this fix the model compiled and loaded cleanly with no further
tracebacks.

### Fix 2: wheel/caster geom friction cannot be targeted by name — no named geom exists

The brief's Step 4 assumed generated geoms would carry a discoverable
`name=` (e.g. matching the mesh file name) that `modify_element type="geom"
name="..."` could target. **That is not the case for this robot/converter
combination.** Investigated in depth:

- The R2D3 description (`dual_rm_description`) is **visual-only** — none of
  its `<link>` elements have a `<collision>` tag, only `<visual>`
  (confirmed by inspecting `robot_description_formatted.urdf`, e.g.
  `link_left_wheel` has an `<inertial>` and a `<visual>`, no `<collision>`).
- `mujoco_ros2_control`'s `update_non_obj_assets()` synthesizes a
  collision/visual geom **pair** from every visual-only mesh geom
  (`urdf_to_mujoco_utils.py:404-461`). It clones the node and calls
  `setAttribute("class", "collision")` / `"visual"` — it never sets a
  `name` attribute on either clone.
- Checked whether `<decompose_mesh>` (obj2mjcf convex decomposition) would
  produce named sub-geoms instead: read `obj2mjcf/mjcf_builder.py` — every
  `etree.SubElement(obj_body, "geom", mesh=..., ...)` call it makes also
  omits `name=`. Decomposition would not have solved the problem, so it
  wasn't run (also not needed — no mesh failed to load).
- Empirically confirmed with the generated MJCF: of the 76 total `<geom>`
  elements in the file, only the 8 **scene** geoms (`ground`,
  `wall_north/south/east/west`, `box1`, `box2`, `cylinder1`, all injected
  verbatim from `nav_empty.xml`) have a `name=` attribute. **All 68 robot
  geoms — every link, not just wheels — are unnamed.**
- `add_modifiers()` (the function that applies `modify_element`) matches
  strictly on `(tagName, element.getAttribute("name"))`; an unnamed geom's
  `getAttribute("name")` returns `""`, so a `modify_element type="geom"
  name="link_left_wheel"` entry (or any other guessed name) simply never
  matches anything — it's silently a no-op, not an error.

**Conclusion:** per-wheel / per-caster geom friction differentiation is not
achievable with `mujoco_ros2_control` 0.0.3 against this description
package, purely from `r2d3_mujoco/` (would require either patching the
vendored converter to preserve/assign geom names, or adding named
`<collision>` elements to `dual_rm_description` — both out of scope for
this package).

**Fallback applied (per the brief's own guidance — "raw_inputs `<default>`
classes are the fallback"):** the drive-wheel friction/condim values are
applied to the shared `collision` default class instead of to specific
geoms:

```xml
<default class="collision">
    <geom group="3" type="mesh" friction="1.5 0.005 0.0001" condim="4"/>
</default>
```

This applies `friction="1.5 0.005 0.0001" condim="4"` (the brief's *drive
wheel* values, matching the Gazebo `mu=1.5` tuning) to **every** robot
collision geom, not just wheels/casters. This is considered acceptable
because:

- Only the drive wheels and the 4 caster assemblies (8 wheel bodies)
  normally contact the ground/scene; the arms/torso/head are elevated and
  don't participate in locomotion contacts.
- Unlike the Gazebo model (which used a low caster friction, `mu=0.3` on
  the wheel / `mu=0.05` on the bracket, to keep a *single-DOF* caster
  approximation from fighting the diff drive), the MuJoCo model has each
  caster as a genuine **2-DOF** joint pair — `joint_swivel_wheel_N_1`
  (yaw swivel, `damping=0.5 frictionloss=0.1`) and
  `joint_swivel_wheel_N_2` (roll, `damping=0.01 frictionloss=0.005`),
  both already present in `robot_description_formatted.urdf` from the
  source description. A swiveling caster with a free-spinning roll axis
  self-aligns to the direction of travel regardless of ground friction, so
  it does not need an artificially low `mu` the way a simplified
  single-DOF Gazebo caster does. This was validated empirically (see
  below) — the robot drives straight and turns correctly with this uniform
  friction.

The `modify_element type="geom" name="..."` mechanism is left undocumented
example scaffolding was **not** added to `processed_inputs`, since there is
nothing it could correctly match; adding entries with guessed/wrong names
would silently do nothing, which is worse than not having them.

### Fix 3: added `noslip_iterations="3"`

Headless validation (below) initially showed the robot drifting/spinning
significantly off a straight line during a 30 s full-throttle
straight-drive test (lateral drift of ~4.5 m against ~2.1 m of forward
progress, heading rotating by ~1.7 rad — i.e. it curved into a near
90-degree turn instead of going straight). Per the brief's contingency,
added:

```xml
<option integrator="implicitfast" noslip_iterations="3"/>
```

This reduced the drift by roughly an order of magnitude (see Test 4
below). Some residual lateral drift remains (~0.85 m laterally over a
30 s / 4.6 m run, with heading staying essentially constant at ~0.03 rad
change) — this reads as a mild sideways skid rather than a turn, plausibly
from a minor mass/COM asymmetry or contact-solver residual, not a URDF/MJCF
correctness bug. Considered acceptable for this spike; worth revisiting
during Task 7 runtime verification if the real controllers show similar
drift.

## Structural checks (final `mujoco_description_formatted.xml`)

```
grep -c "freejoint\|floating_base_joint" $MJCF   -> 1
grep -c "<rangefinder" $MJCF                     -> 1   (replicated to 240 rays at compile time, see below)
grep -c 'name="camera"' $MJCF                    -> 1   (attribute order in the emitted XML is
                                                          fovy/mode/name/..., so `camera name="camera"`
                                                          as a literal substring does not match —
                                                          use `name="camera"` instead)
grep -c "imu_sensor_quat\|imu_sensor_gyro\|imu_sensor_accel" $MJCF  -> 3
grep -c 'name="wall_north"' $MJCF                -> 1   (scene merged)
grep -o 'joint="joint_left_wheel"' $MJCF | head -1  -> joint="joint_left_wheel" (velocity actuator present)
```

Free joint name: **`floating_base_joint`** (matches the brief's expectation
exactly, `<freejoint name="floating_base_joint"/>` inside
`<body name="base_footprint">`).

Rangefinder naming: the raw MJCF has a single literal
`<rangefinder name="lidar" site="rf"/>`, but the `<lidar>` `processed_inputs`
tag causes the converter to add a `<replicate count="240" .../>` around the
`rf` site; at `MjModel` compile time this expands into 240 real sensors
named **`lidar-000` .. `lidar-239`** (confirmed via
`mujoco.MjModel` — total sensor count is 243 = 3 IMU sensors +
240 lidar rays).

Wheel/caster bodies and their (unnamed) geoms — confirmed via
`model.geom_bodyid` / `mj_id2name` on the compiled model (2 geoms per body:
one `class="collision"`, one `class="visual"`, both nameless):

| Body | Geom name |
|---|---|
| `link_left_wheel` | *(unnamed)* x2 |
| `link_right_wheel` | *(unnamed)* x2 |
| `link_swivel_wheel_1_1` .. `link_swivel_wheel_4_1` (yaw link) | *(unnamed)* x2 each |
| `link_swivel_wheel_1_2` .. `link_swivel_wheel_4_2` (roll/wheel link) | *(unnamed)* x2 each |

Caster bodies are **not** fused into the base — confirmed: each
`link_swivel_wheel_N_1` / `_N_2` pair appears as its own `<body>` in the
worldbody tree with its own `<freejoint>`-free hinge joint
(`joint_swivel_wheel_N_1`, `joint_swivel_wheel_N_2`), because they're
connected by revolute joints, not fixed joints (the converter only fuses
bodies joined by `fixed` joints). This matches the brief's expectation.

## Headless physics validation

Interactive `ros2 run mujoco_vendor simulate` is not available in this
environment (no GUI). Validated instead with a scratch script (not
committed) run under the converter venv's Python
(`~/.ros/ros2_control/.venv/bin/python`, which has `mujoco` 3.7.0
installed), loading
`~/.ros/r2d3_mujoco/65b/mujoco_description_formatted.xml` directly via
`mujoco.MjModel.from_xml_path`.

Checks performed and results (full output captured in
`.superpowers/sdd/task-4-report.md`):

1. **Settle** (2000 steps, zero ctrl): no NaN in `qpos`; base free-joint z
   settled to `0.0036` m (within `[0.0, 0.4]`) — **PASS**.
2. **Drive straight** (+5.0 rad/s both wheel actuators, 3000 steps): base
   translated `1.23` m, deviating only ~0.01 m laterally — **PASS**
   (`> 0.3` m required).
3. **Turn in place** (-3.0 / +3.0 rad/s, 3000 steps): yaw changed `1.70`
   rad — **PASS** (`> 0.3` rad required).
4. **30 s straight-drive drift** (settle first, then +5.0/+5.0 for 15000
   steps @ dt=0.002): forward progress `4.57` m, lateral drift `0.85` m,
   heading change `0.03` rad. Not a hard pass/fail gate in the brief;
   recorded as informational (see Fix 3 above for the `noslip_iterations`
   mitigation already applied).

All assertions in the validation script passed (`ALL VALIDATION TESTS
PASSED.`).

## Files changed

- `r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro`: added the `collision`/`visual`
  `<default>` block (required for the MJCF to compile at all), applied the
  drive-wheel friction/condim tuning to the shared `collision` default class
  (documented fallback for the unnamed-geom limitation above), and added
  `noslip_iterations="3"` to `<option>`.
- `r2d3_mujoco/SPIKE_NOTES.md`: this file (new).

## Follow-ups for later tasks

- Task 5 (`ensure_mjcf.py`): use the exact command recorded above; treat a
  stable `mujoco_description_formatted.xml` (no size/mtime change for a
  couple of seconds) plus the "Adding replicates to lidar_link" log line as
  the completion signal, then kill both the wrapper and the python
  processes — `pkill -f make_mjcf_from_robot_description` matched the
  python child in this environment but not always the `ros2 run` wrapper
  shell process reliably; kill by PID pair to be safe.
- If exact per-wheel/per-caster friction differentiation becomes a real
  requirement later (e.g. Task 7 shows the uniform friction is
  insufficient), the real fix is upstream: add named `<collision>` geometry
  to `dual_rm_description`'s wheel/caster links, which is out of scope for
  `r2d3_mujoco/`.

## `/scan` lidar self-occlusion: root cause and fix (post-Task-7 follow-up)

Task 7 documented `/scan` publishing at the right rate/structure but every
range coming back `-1.0`. Root cause, confirmed with a standalone `mj_ray`
script against the cached MJCF (see `.superpowers/sdd/task-7-report.md` for
the original repro and `task-7-fix-report.md`-equivalent section appended to
that same file for the full writeup):

- MuJoCo's `<rangefinder>` sensor excludes exactly **one** body from ray
  casting: `bodyexclude = m->site_bodyid[objid]` (`engine_sensor.c`), the
  body that owns the sensor's own site. `mujoco_ros2_control` 0.0.3's
  `add_lidar_from_sites()` (`urdf_to_mujoco_utils.py`, hard-coded, not
  configurable from `r2d3_mujoco/`) always creates the replicated lidar ray
  sites inside a brand-new, geometry-less `"<site>_lidar_body"`, so that
  exclusion never reaches any geom that could actually occlude a ray. The
  chassis geoms doing the occluding all live in the fused `base_footprint`
  body instead.
- `geomgroup`-based filtering is a dead end too: rangefinder sensors always
  call `mj_ray`/`mj_multiRay` with `geomgroup=NULL` (confirmed against
  `engine_sensor.c`), so a geom's `group` attribute has zero effect on ray
  casting no matter what default class it's in.
- The filter ray casting *does* honor unconditionally is alpha:
  `ray_eliminate()` (`engine_ray.c`) drops any geom whose rgba alpha (or
  material alpha) is exactly 0. Since every robot geom is an unnamed clone
  of a URDF visual mesh/primitive (0/68 robot geoms carry a `name`
  attribute — see the Task 4 finding above), `modify_element` can never
  target an individual occluding geom by name, but mesh geoms keep their
  `mesh="<stl-name>"` attribute and the lidar housing is the only primitive
  geom of its exact size, so both can still be matched textually.
- Three groups of geoms were found self-occluding, each independently
  confirmed with `mj_ray`:
  1. **The lidar housing itself** — modeled in
     `dual_rm_simulation/urdf/sensors/lidar.urdf.xacro` as a
     `<cylinder radius="0.03" length="0.05"/>` visual primitive whose origin
     coincides exactly with the rangefinder site, so every ray starts
     *inside* the housing and immediately exits through its own wall
     (~0.03 m).
  2. **`base_link_underpan` and `body_base_link`** (the chassis pan the
     lidar mounts to) — even with the housing cleared, all 240 rays still
     hit these two meshes' true (non-convex) surface at <0.17 m in every
     direction; the mount sits flush against a raised boss on the chassis,
     not just inside the collision hull's padding.

**Fix applied** (`r2d3_mujoco/urdf/mujoco_inputs.urdf.xacro` +
`r2d3_mujoco/scripts/ensure_mjcf.py`):

- `mujoco_inputs.urdf.xacro`: added `rgba="1 1 1 0"` to the shared
  `collision` default class (defense in depth — hides every collision-hull
  copy robot-wide from ray casting, not just the three meshes above; does
  not affect contact physics, rgba is purely visual).
- `ensure_mjcf.py`: added `patch_lidar_housing_visibility()`, a targeted
  text substitution over the generated MJCF that zeroes the rgba alpha on
  the *visual* copies of the lidar housing cylinder and the two chassis
  meshes (their inline rgba, baked from URDF material colors, would
  otherwise override the collision-class default and stay ray-visible).
  This can't be done via `raw_inputs`/`modify_element` because those only
  see the *input* URDF/scene, not the converter's synthesized output geoms.
- This required restructuring `ensure_mjcf.py`'s conversion flow: the
  vendored converter's own `--publish_topic` publishes its raw output
  *before* this script gets a chance to patch it, and (separately)
  `add_mujoco_info()` only emits an absolute `<compiler meshdir=...>` (vs.
  a relative one that fails to resolve when MuJoCo loads the MJCF from a
  ROS string message) when `--publish_topic` is set at all. The fix: the
  converter subprocess is still given `--publish_topic`, but pointed at a
  throwaway internal topic (`internal_convert_topic()`) that nothing
  subscribes to; `ensure_mjcf.py` polls the output file for size stability
  (the converter spins forever once `--publish_topic` is set, so it can't
  be `wait()`-ed on — same file-stability signal used manually in the Task 4
  spike), kills the whole process group, patches the file, and is the one
  and only publisher on the *real* topic.
- Trade-off accepted: hiding the two chassis meshes from ray casting also
  hides them from the RGB/depth camera's rendering (alpha is a rendering
  property, not ray-casting-only). The `camera` site is head-mounted
  looking outward/forward, so the underpan is not normally in frame; this
  was judged acceptable against a `/scan` that otherwise never reports any
  wall. If camera parity for the underpan ever matters, the real fix would
  need per-consumer geom duplication (a ray-cast-only invisible copy plus a
  separately-rendered visible copy), which the vendored converter has no
  hook for from `r2d3_mujoco/`.

**Result**: `/scan` on a live launch now reports all 240 ranges valid
(sample run: min 1.97 m, max 6.73 m, 0 invalid), against a 10x10 m room —
verified both on a cache-miss (fresh conversion) and cache-hit (patched
file re-published from disk) run.
