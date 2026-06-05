"""Minimal V1 scene: ground plane + R2D3 USD reference.

Tables and manipulable objects belong to the *task* layer (engine + YAML),
not bring-up — keep this module dumb on purpose so M1 can be verified
without any per-task knowledge.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


# Authoring choices that match the URDF audit (docs/urdf_audit.md):
#   - Up axis is +Z
#   - Stage units are meters
#   - World frame origin sits at the robot's base_link_underpan when the robot
#     is loaded at the origin with no transform.
DEFAULT_USD = (
    Path(__file__).resolve().parents[1] / "usd" / "r2d3_v1.usda"
)
ROBOT_PRIM_PATH = "/r2d3_v1"


def assemble(world, *, usd_path: Optional[Path] = None) -> str:
    """Add a ground plane and load the R2D3 USD into the stage.

    Must be called after the SimulationApp has booted (so omni / isaacsim
    imports work) but before ``world.reset()`` — the reset() then
    initializes physics handles for everything loaded here.

    Returns
    -------
    str
        The stage path the robot was loaded at. Pass this to ``Robot`` so
        it can bind the Articulation view to the right prim.
    """
    # Local imports — only valid after SimulationApp is up.
    from isaacsim.core.api.objects.ground_plane import GroundPlane
    from isaacsim.core.utils.stage import add_reference_to_stage

    if usd_path is None:
        usd_path = DEFAULT_USD
    usd_path = Path(usd_path).resolve()
    if not usd_path.is_file():
        raise FileNotFoundError(
            f"R2D3 USD not found at {usd_path}. "
            f"Render it first with `bash isaac_sim/urdf/render.sh` "
            f"then `scripts/urdf_to_usd.py`."
        )

    # NB: NO ground plane. The chassis is already fixed to world via
    # PhysicsFixedJoint(``root_joint``) authored in the USD — the robot
    # can't fall. Adding a ground plane causes the AGV wheels to collide
    # with it at step 1, propagating non-finite forces through the whole
    # articulation. Tasks that need a ground plane spawn one in their
    # scene script after lifting the robot above z=0.
    add_reference_to_stage(usd_path=str(usd_path), prim_path=ROBOT_PRIM_PATH)

    _add_lighting()
    configure_articulation_physics()
    return ROBOT_PRIM_PATH


# Articulation root prim (carries PhysicsArticulationRootAPI in the USD).
ARTICULATION_ROOT = f"{ROBOT_PRIM_PATH}/Geometry/base_link_underpan"


def configure_articulation_physics() -> None:
    """Author articulation-level physics props that must be set BEFORE reset.

    Self-collisions OFF (the folded home pose tucks the arms near the column)
    and high solver iteration counts (for stable position drives). These are
    prim-property edits that throw if attempted at runtime, so they live here
    (assemble runs before world.reset()). Drive gains + gravity are set at
    runtime in Robot._configure_drives().
    """
    import omni.usd
    from pxr import PhysxSchema

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(ARTICULATION_ROOT)
    if not root or not root.IsValid():
        logger.warning("articulation root %s not found; skipping physics cfg",
                       ARTICULATION_ROOT)
        return
    papi = PhysxSchema.PhysxArticulationAPI.Apply(root)
    papi.CreateEnabledSelfCollisionsAttr(False)
    papi.CreateSolverPositionIterationCountAttr(32)
    papi.CreateSolverVelocityIterationCountAttr(4)
    logger.info("articulation physics: self-collisions off, solver iters (32,4)")

    _author_joint_drives(stage)


def _author_joint_drives(stage) -> None:
    """Author UsdPhysics.DriveAPI on every revolute/prismatic joint.

    The urdf_usd_converter authors NO drives, so the joints are free (drift /
    NaN) and runtime set_gains is a no-op (nothing to configure). Authoring a
    DriveAPI with stiffness+damping CREATES the position drive so the joints
    track commanded targets. Gains come from sim_topics.DRIVE_GAINS.
    """
    from pxr import UsdPhysics
    from . import sim_topics as t

    n = 0
    for prim in stage.Traverse():
        is_rev = prim.IsA(UsdPhysics.RevoluteJoint)
        is_prism = prim.IsA(UsdPhysics.PrismaticJoint)
        if not (is_rev or is_prism):
            continue
        name = prim.GetName()
        kp, kd = t.DRIVE_GAINS[t.drive_group(name)]
        drive_axis = "angular" if is_rev else "linear"
        drive = UsdPhysics.DriveAPI.Apply(prim, drive_axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(float(kp))
        drive.CreateDampingAttr().Set(float(kd))
        drive.CreateMaxForceAttr().Set(float(t.DRIVE_MAX_FORCE))
        n += 1
    logger.info("authored DriveAPI on %d joints", n)


def add_visual_box(prim_path: str, center, size, color) -> str:
    """Author a visual-only colored box (UsdGeom.Cube + displayColor).

    No rigid body, no collider — safe to add any time (no physics init,
    can't perturb the robot articulation). Used for preview props (ground,
    workbench, cube) in the capture tool. The real eval scene spawns
    physics-interactive objects from task YAML; that comes with the engine.
    """
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(1.0)                       # unit cube: extent -0.5..0.5
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    xf.AddScaleOp().Set(Gf.Vec3f(float(size[0]), float(size[1]), float(size[2])))

    # Bind a UsdPreviewSurface material. displayColor alone renders BLACK under
    # RTX on faces not hit by a direct light (no ambient from primvar color); a
    # material albedo lights correctly from any angle. Wrapped in try/except so
    # a USD-version API mismatch can never crash the caller — it just falls back
    # to displayColor.
    try:
        from pxr import Sdf, UsdShade
        mtl = UsdShade.Material.Define(stage, f"{prim_path}/Mat")
        shader = UsdShade.Shader.Define(stage, f"{prim_path}/Mat/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        surf_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        mtl.CreateSurfaceOutput().ConnectToSource(surf_out)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mtl)
    except Exception as e:  # noqa: BLE001
        logger.warning("material bind failed for %s (%s); using displayColor", prim_path, e)
    return prim_path


def world_range(prim_path: str):
    """Return (min np[3], max np[3]) world-space AABB of a prim subtree."""
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    return (np.asarray([mn[0], mn[1], mn[2]], dtype=float),
            np.asarray([mx[0], mx[1], mx[2]], dtype=float))


def _add_lighting() -> None:
    """Add a dome + distant key light.

    An RTX scene with no lights renders pure black RGB (depth still works
    because it's geometric). The default ground plane used to carry a
    light; since we don't add one, author lights explicitly. Benefits both
    the headless capture and the interactive VNC viewer.
    """
    import omni.usd
    from pxr import Gf, Sdf, UsdLux

    stage = omni.usd.get_context().get_stage()

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
    dome.CreateIntensityAttr(1200.0)
    # Slight blue-grey so the background isn't pure white (a white robot reads
    # better against it); the key light below supplies shape definition.
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.90))

    from pxr import UsdGeom

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/KeyLight"))
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    # Tilt the key light down and to one side for shape definition.
    xf = UsdGeom.Xformable(key.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))

    # Fill light from the front (+X side, angled down) so faces pointing toward
    # the robot / head camera aren't left pitch-black (displayColor prims get no
    # ambient under RTX — only direct light). Lower intensity than the key.
    fill = UsdLux.DistantLight.Define(stage, Sdf.Path("/FillLight"))
    fill.CreateIntensityAttr(1500.0)
    fill.CreateAngleAttr(2.0)
    xff = UsdGeom.Xformable(fill.GetPrim())
    xff.ClearXformOpOrder()
    # Point roughly toward -X and down: light comes from the +X/front, lighting
    # the table faces the head camera sees.
    xff.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 0.0, 200.0))
