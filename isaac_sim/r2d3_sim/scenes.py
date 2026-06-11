"""Training environments for R2D3.

Load a prebuilt NVIDIA Isaac environment (warehouse) or compose a furnished room
(kitchen, living room) — each populated with **manipulable objects** so users can
start manipulation work immediately. Assets stream from the Isaac **cloud** asset
server (auto-detected via ``get_assets_root_path``); the machine needs internet.

    from isaac_sim.r2d3_sim import R2D3, scenes
    sim = R2D3(mobile=True, setup=lambda w: scenes.load("kitchen", w))
    manifest = scenes.load(...)   # also returned from the setup hook

`load(name, world)` builds the scene and returns a manifest dict:
    spawn (x, y, yaw_deg)         - where to stand the robot (caller positions it)
    look / eye (x, y, z)          - suggested camera target / position (optional)
    objects {name: prim_path}     - manipulable rigid-body objects in the scene
    surface_z {name: float|None}  - the surface each object rests on (None = scene floor)

Surfaces (counters, island, coffee table) are **collidable** `add_fixed_box`es, so
objects rest on them. Room floors/walls are visual-only (the robot base is held
kinematically; a collidable ground would perturb the articulation). See
isaac_sim/tests/diag_scenes.py (`--check`) for the renderer + interactability check.
"""
from __future__ import annotations

# Fallback Isaac cloud assets root (used if get_assets_root_path is unavailable).
_S3 = "https://omniverse-content-production.s3.amazonaws.com/Assets/Isaac/6.0"
_OFF = "/Isaac/Environments/Office/Props"
_YCB = "/Isaac/Props/YCB/Axis_Aligned"


def asset_root() -> str:
    """The Isaac assets root (``.../Assets/Isaac/<ver>``)."""
    for mod in ("isaacsim.storage.native", "isaacsim.core.utils.nucleus"):
        try:
            m = __import__(mod, fromlist=["get_assets_root_path"])
            r = m.get_assets_root_path()
            if r:
                return r
        except Exception:  # noqa: BLE001
            pass
    return _S3


def _ref(usd, prim_path, pos=(0.0, 0.0, 0.0), yaw=0.0, scale=1.0):
    """Reference a USD at a pose (yaw degrees about +Z). The asset is referenced
    under a *wrapper* Xform we transform (at ``prim_path/ref``), leaving the asset's
    own internal transforms intact (clearing them directly drops the geometry to a
    wrong origin). The ``/ref`` child also marks "this is a referenced asset" for the
    floor-snap pass in diag_scenes."""
    from isaacsim.core.utils.stage import add_reference_to_stage
    import omni.usd
    from pxr import UsdGeom, Gf
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, prim_path)                 # wrapper we own
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    if yaw:
        xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    add_reference_to_stage(usd_path=usd, prim_path=prim_path + "/ref")   # asset untouched
    return prim_path


def _room(size=(7.0, 7.0), wall_h=2.8, floor=(0.46, 0.42, 0.38), wall=(0.58, 0.56, 0.54),
          dome=55.0, key=1500.0, fill=950.0):
    """An L of two walls + a floor with interior lighting — a backdrop for composed
    scenes (the prebuilt envs bring their own). Visual-only (no colliders)."""
    from . import scene as scene_mod
    from . import helpers as h
    sx, sy = size
    scene_mod.add_visual_box("/World/room/floor", (0.0, 0.0, -0.01), (sx, sy, 0.02), floor)
    scene_mod.add_visual_box("/World/room/wall_x", (-sx / 2, 0.0, wall_h / 2), (0.12, sy, wall_h), wall)
    scene_mod.add_visual_box("/World/room/wall_y", (0.0, sy / 2, wall_h / 2), (sx, 0.12, wall_h), wall)
    h.set_lighting(dome=dome, key=key, fill=fill)


# --------------------------------------------------------------- manipulable objects
def _manifest(spawn, look=None, eye=None):
    m = {"spawn": spawn, "objects": {}, "surface_z": {}}
    if look is not None:
        m["look"] = look
    if eye is not None:
        m["eye"] = eye
    return m


def _ensure_physics(root_path, mass):
    """Make a static USD subtree a dynamic rigid body if it isn't one already
    (idempotent): RigidBodyAPI + MassAPI on the root + CollisionAPI/convex-hull on
    each gprim. YCB/Props physics-readiness varies across assets, so always check."""
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(root)):
        return
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(float(mass))
    for p in Usd.PrimRange(root):
        if p.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(p)
            try:
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set("convexHull")
            except Exception:  # noqa: BLE001
                pass


def add_object(manifest, name, usd, pos, surface_z=None, yaw=0.0, scale=1.0, mass=0.2):
    """Reference a manipulable object under ``/World/objs/<name>``, make it a dynamic
    rigid body, and record it in the manifest. Spawn it slightly above its support
    surface and let physics settle it during reset/stepping."""
    import omni.usd
    from pxr import UsdGeom, Gf
    from . import scene as scene_mod
    prim = f"/World/objs/{name}"
    _ref(asset_root() + usd, prim, pos=pos, yaw=yaw, scale=scale)
    # unit guard: a cm-authored asset comes in ~100x too big -> scale it down
    try:
        lo, hi = scene_mod.world_range(prim)
        if float(max(hi - lo)) > 0.6:
            UsdGeom.Xformable(omni.usd.get_context().get_stage().GetPrimAtPath(prim)) \
                .AddScaleOp(opSuffix="unit").Set(Gf.Vec3f(0.01, 0.01, 0.01))
    except Exception:  # noqa: BLE001
        pass
    _ensure_physics(prim + "/ref", mass)
    manifest["objects"][name] = prim
    manifest["surface_z"][name] = surface_z
    return prim


def load_warehouse(world):
    """NVIDIA's prebuilt full warehouse + cardboard boxes and a tote near the spawn.
    Brings its own lighting, so we don't add a room."""
    import numpy as np
    import omni.usd
    from isaacsim.core.api.objects import DynamicCuboid
    from . import scene as scene_mod
    # The single-shelf warehouse has open floor at the origin (the full warehouse is
    # densely racked — hard to place the robot headless). Floor is at z=0, so the base
    # is re-pinned at 0.27 like the composed rooms.
    _ref(asset_root() + "/Isaac/Environments/Simple_Warehouse/warehouse.usd", "/World/Env")
    man = _manifest((0.0, 0.0, 0.0), look=(0.7, 0.0, 0.5), eye=(4.2, -4.0, 2.6))
    man["hold_base"] = True
    CARD = (0.72, 0.56, 0.36)
    stage = omni.usd.get_context().get_stage()
    for nm, (cx, cy, cz), s, m in (("box1", (1.0, 0.3, 1.0), 0.40, 0.6),       # in front of the robot (+X)
                                   ("box2", (1.1, -0.4, 1.0), 0.30, 0.4),
                                   ("box3", (0.9, 0.7, 1.0), 0.25, 0.3)):
        p = f"/World/objs/{nm}"
        DynamicCuboid(prim_path=p, name=nm, position=np.array([cx, cy, cz]),
                      scale=np.array([s, s, s]), color=np.array(CARD), mass=m,
                      physics_material=scene_mod._phys_material())
        scene_mod._bind_preview_material(p, stage.GetPrimAtPath(p), CARD)
        man["objects"][nm] = p
        man["surface_z"][nm] = None      # rests on the warehouse floor (z=0)
    add_object(man, "tote", "/Isaac/Props/KLT_Bin/small_KLT.usd", (0.8, -0.8, 1.0), mass=1.0)
    return man


def load_kitchen(world):
    """A kitchen of collidable primitive cabinetry (counter run + island + fridge +
    upper cabinets + stovetop) with utensils + groceries on the island/counter.
    Primitives are used for the cabinetry because the SimReady residential kitchen
    USDs don't render reliably headless."""
    import omni.usd
    from pxr import UsdGeom
    from . import scene as scene_mod
    UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World/k")
    _room(size=(7.0, 7.0), floor=(0.80, 0.78, 0.74), wall=(0.88, 0.86, 0.82),
          dome=75.0, key=2000.0, fill=1300.0)
    fbox = scene_mod.add_fixed_box
    CAB = (0.74, 0.60, 0.46); TOP = (0.90, 0.89, 0.86); STEEL = (0.72, 0.74, 0.78); DARK = (0.16, 0.16, 0.18)
    fbox("/World/k/base", (-2.85, 0.0, 0.45), (0.60, 4.0, 0.90), CAB)
    fbox("/World/k/top", (-2.82, 0.0, 0.925), (0.68, 4.0, 0.05), TOP)         # countertop (collidable)
    fbox("/World/k/upper", (-2.98, 0.0, 1.75), (0.34, 4.0, 0.70), CAB)        # wall cabinets
    fbox("/World/k/stove", (-2.82, -1.4, 0.955), (0.64, 0.7, 0.02), DARK)     # stovetop inset
    fbox("/World/k/fridge", (-2.80, 2.7, 0.95), (0.72, 0.72, 1.90), STEEL)    # fridge at the end
    # The island is a low worktop (top 0.85) so the robot's resting arms clear it at
    # the close grasp spawn; the counter run stays at 0.90 (the robot is far from it).
    fbox("/World/k/island", (-1.1, 0.0, 0.40), (1.00, 1.6, 0.80), CAB)
    fbox("/World/k/islandtop", (-1.1, 0.0, 0.825), (1.08, 1.68, 0.05), TOP)   # island worktop (collidable)
    _ref(asset_root() + _OFF + "/SM_Plant01.usd", "/World/k/plant", pos=(2.9, 2.9, 0))
    # robot stands just off the island; the mug is the grasp target for example 08.
    # lift 0.90 keeps the resting arms above the (collidable) island worktop.
    man = _manifest((-0.15, 0.0, 180.0), look=(-1.2, 0.0, 0.9), eye=(4.0, -3.0, 2.4))
    man["lift"] = 0.90
    man["hold_base"] = True       # visual-only floor -> re-pin the base each step
    ISL = 0.85   # island worktop surface; CNT = counter-run top
    CNT = 0.95
    add_object(man, "mug", _YCB + "/025_mug.usd", (-0.67, 0.21, ISL + 0.10), surface_z=ISL, mass=0.12)
    add_object(man, "bowl", _YCB + "/024_bowl.usd", (-0.95, -0.30, ISL + 0.10), surface_z=ISL, mass=0.15)
    add_object(man, "cracker_box", _YCB + "/003_cracker_box.usd", (-1.35, 0.40, ISL + 0.15), surface_z=ISL, yaw=20, mass=0.30)
    add_object(man, "soup_can", _YCB + "/005_tomato_soup_can.usd", (-1.30, -0.45, ISL + 0.10), surface_z=ISL, mass=0.35)
    add_object(man, "mustard", _YCB + "/006_mustard_bottle.usd", (-2.75, 0.6, CNT + 0.12), surface_z=CNT, mass=0.40)
    return man


def load_living_room(world):
    """A living room of Office furniture props (sofa, armchairs, plants, cupboard)
    around a collidable coffee table on a rug, with a few items on the table."""
    import omni.usd
    from pxr import UsdGeom
    from . import scene as scene_mod
    O = asset_root() + _OFF
    UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World/lr")
    _room(size=(7.5, 7.5), floor=(0.40, 0.31, 0.25), wall=(0.60, 0.58, 0.55))
    scene_mod.add_visual_box("/World/lr/rug", (-0.9, 0.0, 0.006), (3.2, 2.6, 0.012), (0.34, 0.24, 0.24))
    _ref(O + "/SM_Sofa.usd", "/World/lr/sofa", pos=(-3.0, 0.0, 0), yaw=90)
    _ref(O + "/SM_Armchair.usd", "/World/lr/arm1", pos=(-0.9, 2.3, 0), yaw=205)
    _ref(O + "/SM_Armchair.usd", "/World/lr/arm2", pos=(-0.9, -2.3, 0), yaw=-25)
    # collidable coffee table (replaces SM_A4_Table, whose collider is unknown)
    scene_mod.add_fixed_box("/World/lr/coffee_top", (-1.5, 0.0, 0.42), (0.95, 0.6, 0.06), (0.46, 0.33, 0.22))
    scene_mod.add_fixed_box("/World/lr/coffee_leg", (-1.5, 0.0, 0.195), (0.80, 0.45, 0.39), (0.40, 0.28, 0.18))
    _ref(O + "/SM_Plant01.usd", "/World/lr/plant1", pos=(-3.3, 3.2, 0))
    _ref(O + "/SM_Plant02.usd", "/World/lr/plant2", pos=(-3.3, -3.2, 0))
    _ref(O + "/SM_Cupboard.usd", "/World/lr/cupboard", pos=(2.6, 3.2, 0), yaw=180)
    man = _manifest((1.4, 0.0, 180.0), look=(-1.2, 0.0, 0.8), eye=(4.6, -3.4, 2.6))
    man["hold_base"] = True       # visual-only floor -> re-pin the base each step
    TBL = 0.45   # coffee-table top surface
    add_object(man, "banana", _YCB + "/011_banana.usd", (-1.35, 0.12, TBL + 0.10), surface_z=TBL, mass=0.10)
    add_object(man, "mug", _YCB + "/025_mug.usd", (-1.62, -0.12, TBL + 0.10), surface_z=TBL, mass=0.12)
    add_object(man, "sugar_box", _YCB + "/004_sugar_box.usd", (-1.62, 0.18, TBL + 0.12), surface_z=TBL, yaw=35, mass=0.30)
    return man


_SCENES = {"warehouse": load_warehouse, "kitchen": load_kitchen, "living_room": load_living_room}


def names():
    return list(_SCENES)


def load(name, world):
    """Build the named environment in ``world``; returns the manifest (see module doc)."""
    if name not in _SCENES:
        raise ValueError(f"unknown scene {name!r}; choose from {names()}")
    return _SCENES[name](world)
