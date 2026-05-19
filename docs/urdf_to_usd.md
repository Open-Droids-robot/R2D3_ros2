# URDF → USD Conversion (Isaac Sim 5.1)

> Research notes captured 2026-05-19, sourced from the `isaacsim.asset.importer.urdf` extension's public source on GitHub. Used to plan the conversion script — actual conversion can only run inside the Isaac Sim container, which is blocked on `/usr0` cleanup.

## Extension namespace

For Isaac Sim **5.x** (we're targeting 5.1.0), the URDF importer extension is:

```
isaacsim.asset.importer.urdf
```

(Renamed from Isaac Sim 4.x's `omni.importer.urdf`. Both share the same command-name surface, only the import path differs.)

The extension is shipped with Isaac Sim — no `omni.kit.app` enable step required when running through `simulation_app.py` or the Isaac Sim GUI. If running headless via Kit, ensure it's listed under `--enable isaacsim.asset.importer.urdf`.

## Public commands

All commands are registered with `omni.kit.commands` and invoked via `omni.kit.commands.execute(<name>, **kwargs)`. They return `(status: bool, result: Any)`.

| Command | Purpose | Returns |
|---|---|---|
| `URDFCreateImportConfig` | Create a fresh `ImportConfig` with defaults | `_urdf.ImportConfig` |
| `URDFParseFile` | Parse a URDF file → in-memory robot | `_urdf.UrdfRobot` |
| `URDFParseText` | Parse a URDF string → in-memory robot | `_urdf.UrdfRobot` |
| `URDFImportRobot` | Import a parsed `UrdfRobot` to a USD stage | str (prim path) |
| `URDFParseAndImportFile` | One-shot: parse file + write USD | str (prim path) |

For our pipeline `URDFParseAndImportFile` is the right entry point.

### Signature (from `source/extensions/isaacsim.asset.importer.urdf/python/impl/commands.py`)

```python
class URDFParseAndImportFile:
    def __init__(self,
                 urdf_path: str = "",
                 import_config = _urdf.ImportConfig(),
                 dest_path: str = "",
                 get_articulation_root: bool = False) -> None:
        ...

    def do(self) -> str:
        # parses urdf_path, creates USD at dest_path if missing,
        # imports robot into the stage, returns the resulting prim path.
        ...
```

## `ImportConfig` fields (complete reference)

Every field exposed by the pybind11 binding in `bindings/isaacsim.asset.importer.urdf/IsaacsimAssetImporterUrdfBindings.cpp`:

| Field | Type | Default behavior | What it does |
|---|---|---|---|
| `merge_fixed_joints` | bool | False | Collapse links connected by fixed joints into the parent (reduces articulation depth). |
| `convex_decomp` | bool | False | Decompose convex meshes into multiple smaller convexes for tighter collision fit. Slow. |
| `import_inertia_tensor` | bool | True | Use URDF-supplied inertia (else identity). Set False only if URDF inertias are bogus. |
| `fix_base` | bool | True | Create a fixed joint pinning base link to world. **Set False for mobile robots.** |
| `self_collision` | bool | False | Enable self-collisions between links in the same articulation. |
| `density` | float | 0.0 | Default density for autocomputed inertia (kg/m³). `0` = autocompute from URDF mass. |
| `default_drive_type` | int (enum) | `JOINT_DRIVE_POSITION` (1) | Drive type for joints lacking one. 0=NONE, 1=POSITION, 2=VELOCITY. |
| `subdivision_scheme` | int (enum) | — | Subdivision for mesh normals. |
| `default_drive_strength` | float | 1e7 | Joint stiffness when drive type = position/velocity and URDF doesn't author one. |
| `default_position_drive_damping` | float | 1e5 | Joint damping for position drives. |
| `distance_scale` | float | 1.0 | Unit scaling. **1.0 = URDF is in meters** (our case), 100.0 = cm. |
| `up_vector` | Vec3 | (0, 0, 1) | Up axis. Z-up matches our URDF. |
| `create_physics_scene` | bool | True | Add a PhysicsScene prim if the stage doesn't have one. |
| `make_default_prim` | bool | True | Set the imported robot as the stage's default prim. |
| `collision_from_visuals` | bool | False | Generate convex collisions from visual meshes. **False** keeps explicit URDF collision meshes. |
| `replace_cylinders_with_capsules` | bool | False | Replace cylinder primitives with capsules (faster contact). |
| `parse_mimic` | bool | True | Honor URDF `<mimic>` tags using **PhysX Tendons**. **REQUIRED for our gripper.** |
| `override_joint_dynamics` | bool | False | Replace URDF-authored joint dynamics with defaults. |

### Enums

```
UrdfJointTargetType        UrdfNormalSubdivisionScheme
─────────────────         ─────────────────────────
JOINT_DRIVE_NONE = 0      (values not exposed in headers we read)
JOINT_DRIVE_POSITION = 1
JOINT_DRIVE_VELOCITY = 2
```

## R2D3-specific config

Recommended values for our V1 conversion of `dual_rm_75b_description.urdf`, with reasoning:

| Field | Value | Reason |
|---|---|---|
| `fix_base` | **False** | R2D3 is mobile (AGV is the base). We'll lock AGV joints separately in the scene, but the URDF base shouldn't be world-fixed. |
| `merge_fixed_joints` | **False** (initial), then maybe True | Start with False so we see the full link tree in Isaac Sim's stage inspector and can verify mounting points (head, hands). Switch to True once we're sure no fixed link will need its own prim later. |
| `convex_decomp` | **False** initially | Fingers and small links should be fine with single convex hulls. Enable only if grasping behavior shows mesh-collision artifacts. |
| `import_inertia_tensor` | **True** | URDF inertias from RealMan should be physically reasonable. |
| `self_collision` | **False** | Enable later on specific link pairs (e.g., gripper fingers vs hand body) via post-import edits, not globally. Global self-collision is expensive and noisy. |
| `density` | `0.0` | Autocompute from URDF mass + volume. |
| `default_drive_type` | `1` (POSITION) | Matches `Movej`/`Movejp` semantics from the real driver. |
| `default_drive_strength` | `1e7` | High stiffness — arms are position-controlled and we want good tracking. |
| `default_position_drive_damping` | `1e5` | Standard 1% of stiffness; revise if oscillation observed. |
| `distance_scale` | `1.0` | URDF uses meters (verified — mesh sizes match RM75-B real dimensions, ~0.6 m arm reach). |
| `up_vector` | `(0.0, 0.0, 1.0)` | Z-up matches URDF and Isaac convention. |
| `create_physics_scene` | `True` | Sane default. |
| `make_default_prim` | `True` | We want `/r2d3` to be the default prim. |
| `collision_from_visuals` | **False** | The URDF has explicit collision STLs separate from visuals; prefer those. |
| `replace_cylinders_with_capsules` | `False` | No cylinder primitives in the R2D3 URDF (everything is STL mesh). |
| `parse_mimic` | **True** | **Critical** for our parallel gripper — drive joint + mimic finger join via PhysX Tendons. |
| `override_joint_dynamics` | `False` | URDF joint dynamics (effort/velocity limits) are mostly correct; don't blow them away. |

## Concrete script outline

A first-pass conversion script is at `scripts/urdf_to_usd.py`. Highlights:

```python
# This MUST run inside Isaac Sim's Python (e.g.,
#   /isaac-sim/python.sh scripts/urdf_to_usd.py)
# It will not run on the host because omni.* / isaacsim.* are not on PYTHONPATH.

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})       # offscreen import

import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf

status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.fix_base = False
cfg.merge_fixed_joints = False
cfg.parse_mimic = True
cfg.distance_scale = 1.0
cfg.up_vector = (0.0, 0.0, 1.0)
cfg.make_default_prim = True
cfg.collision_from_visuals = False
cfg.default_drive_type = 1                        # POSITION
cfg.default_drive_strength = 1e7
cfg.default_position_drive_damping = 1e5
cfg.import_inertia_tensor = True

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_IN,
    import_config=cfg,
    dest_path=USD_OUT,
    get_articulation_root=True,
)
print(f"Imported robot at: {prim_path}")
sim_app.close()
```

## Gotchas / open questions

1. **Commands require Isaac Sim Python**, not host Python — `omni.kit.commands` is part of Kit, only available inside the Isaac Sim container or via Isaac Lab.
2. **`URDFParseAndImportFile` is non-idempotent.** If `dest_path` already exists, it imports into the existing stage rather than recreating it. Workflow: delete the USD first, or use a fresh path each run.
3. **`parse_mimic` uses PhysX Tendons.** The mimic finger joint will move under physics, but the standard `/joint_states` publisher may only report the *drive* joint. We may need to author the joint state publisher to include both finger joints, or use the Isaac Sim ROS2 bridge's joint state action which handles tendons.
4. **Fingers don't exist in the source URDF.** Our V1 plan adds prismatic finger joints *after* import via the Python USD API (`UsdPhysics.PrismaticJoint.Define(...)`), or by editing a wrapper xacro before conversion. The wrapper-xacro approach is cleaner — author once, all imports inherit it.
5. **Articulation root.** R2D3 has multiple potential articulation roots (the AGV vs the dual-arm platform). Pass `get_articulation_root=True` and inspect what gets chosen; we may need to manually set the `ArticulationRootAPI` on `body_base_link` (the dual-arm platform's base) post-import.
6. **Camera mount.** The `sensor_d435` xacro macro isn't in the source URDF (separate package). Either compose it in via xacro before conversion (preferred) or add the camera prim in USD after import.
7. **Convex decomposition** is the most common reason fingers misbehave when grasping small objects. We start with `convex_decomp=False`; if cubes slip through fingers, retry the pinch-region links with True.
8. **Mesh paths.** URDF uses `package://dual_rm_75b_description/meshes/<file>.STL`. The importer needs to resolve `package://` URIs — works if the URDF's parent directory is on the importer's search path. Verify by passing the URDF's containing directory as `urdf_path` (the importer extracts the root from it).

## Source references

- Source repo: <https://github.com/isaac-sim/IsaacSim>
- Commands: `source/extensions/isaacsim.asset.importer.urdf/python/impl/commands.py`
- Bindings: `source/extensions/isaacsim.asset.importer.urdf/bindings/isaacsim.asset.importer.urdf/IsaacsimAssetImporterUrdfBindings.cpp`
- Docs site (UI-centric): <https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html>
