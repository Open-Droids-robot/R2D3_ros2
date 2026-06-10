"""Training environments for R2D3.

Load a prebuilt NVIDIA Isaac environment (warehouse) or compose a furnished room
from the Isaac SimReady / Office asset libraries (kitchen, living room). Assets are
fetched from the Isaac **cloud** asset server, so the machine needs internet
(reachability is auto-detected via ``get_assets_root_path``).

    from isaac_sim.r2d3_sim import R2D3, scenes
    sim = R2D3(mobile=True, setup=lambda w: scenes.load("kitchen", w))

`load(name, world)` adds the environment to the stage and returns a dict with:
    spawn (x, y, yaw_deg)  - where to stand the robot (caller positions it)
    look  (x, y, z)        - suggested camera target (optional)
    eye   (x, y, z)        - suggested camera position (optional)
See isaac_sim/tests/diag_scenes.py for the renderer.
"""
from __future__ import annotations

# Fallback Isaac cloud assets root (used if get_assets_root_path is unavailable).
_S3 = "https://omniverse-content-production.s3.amazonaws.com/Assets/Isaac/6.0"


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
    under a *wrapper* Xform we transform, leaving the asset's own internal
    transforms intact (clearing them directly drops the geometry to a wrong origin)."""
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
    scenes (the prebuilt envs bring their own). Surfaces are mid-toned and lights
    modest so the (white) robot + furniture aren't blown out under RTX."""
    from . import scene as scene_mod
    from . import helpers as h
    sx, sy = size
    scene_mod.add_visual_box("/World/room/floor", (0.0, 0.0, -0.01), (sx, sy, 0.02), floor)
    scene_mod.add_visual_box("/World/room/wall_x", (-sx / 2, 0.0, wall_h / 2), (0.12, sy, wall_h), wall)
    scene_mod.add_visual_box("/World/room/wall_y", (0.0, sy / 2, wall_h / 2), (sx, 0.12, wall_h), wall)
    h.set_lighting(dome=dome, key=key, fill=fill)


_KIT = "/Isaac/SimReady/Residential/Kitchen"
_OFF = "/Isaac/Environments/Office/Props"


def load_warehouse(world):
    """NVIDIA's prebuilt full warehouse (shelves, pallets, forklifts, floor markings).
    Brings its own lighting, so we don't add a room."""
    _ref(asset_root() + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd", "/World/Env")
    # spawn in the open floor area (origin is occluded by racks); the robot settles
    # onto the warehouse floor (height != 0) and the renderer lifts the camera to match.
    return {"spawn": (2.0, -2.0, 30.0), "look": (2.0, -2.0, 0.9), "eye": (5.6, -5.6, 3.3)}


def load_kitchen(world):
    """A kitchen built from clean primitive cabinetry (counter run + island + fridge
    + upper cabinets + stovetop) with a live plant accent. Primitives are used for
    the cabinetry because the SimReady residential kitchen USDs don't render
    reliably headless; swap them back in via `_ref` once that's resolved."""
    from . import scene as scene_mod
    import omni.usd
    from pxr import UsdGeom
    UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World/k")
    _room(size=(7.0, 7.0), floor=(0.80, 0.78, 0.74), wall=(0.88, 0.86, 0.82),
          dome=75.0, key=2000.0, fill=1300.0)
    box = scene_mod.add_visual_box
    CAB = (0.74, 0.60, 0.46); TOP = (0.90, 0.89, 0.86); STEEL = (0.72, 0.74, 0.78); DARK = (0.16, 0.16, 0.18)
    # base counter run along the -X wall (y in [-2, 2]) + overhanging countertop
    box("/World/k/base", (-2.85, 0.0, 0.45), (0.60, 4.0, 0.90), CAB)
    box("/World/k/top", (-2.82, 0.0, 0.925), (0.68, 4.0, 0.05), TOP)
    box("/World/k/upper", (-2.98, 0.0, 1.75), (0.34, 4.0, 0.70), CAB)        # wall cabinets
    box("/World/k/stove", (-2.82, -1.4, 0.955), (0.64, 0.7, 0.02), DARK)     # stovetop inset
    box("/World/k/fridge", (-2.80, 2.7, 0.95), (0.72, 0.72, 1.90), STEEL)    # fridge at the end
    # central island + countertop
    box("/World/k/island", (-1.1, 0.0, 0.45), (1.00, 1.6, 0.90), CAB)
    box("/World/k/islandtop", (-1.1, 0.0, 0.925), (1.08, 1.68, 0.05), TOP)
    # a live SimReady-free prop that renders fine
    _ref(asset_root() + _OFF + "/SM_Plant01.usd", "/World/k/plant", pos=(2.9, 2.9, 0))
    # primitives have no colliders, so the robot can stand right at the island.
    return {"spawn": (0.7, 0.0, 180.0), "look": (-1.2, 0.0, 0.95), "eye": (4.0, -3.0, 2.4)}


def load_living_room(world):
    """A living room composed from Office furniture props: sofa + armchairs around a
    coffee table on a rug, plants in the corners."""
    O = asset_root() + _OFF
    from . import scene as scene_mod
    import omni.usd
    from pxr import UsdGeom
    UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World/lr")   # liftable group
    _room(size=(7.5, 7.5), floor=(0.40, 0.31, 0.25), wall=(0.60, 0.58, 0.55))
    scene_mod.add_visual_box("/World/lr/rug", (-0.9, 0.0, 0.006), (3.2, 2.6, 0.012), (0.34, 0.24, 0.24))
    _ref(O + "/SM_Sofa.usd", "/World/lr/sofa", pos=(-3.0, 0.0, 0), yaw=90)
    _ref(O + "/SM_Armchair.usd", "/World/lr/arm1", pos=(-0.9, 2.3, 0), yaw=205)
    _ref(O + "/SM_Armchair.usd", "/World/lr/arm2", pos=(-0.9, -2.3, 0), yaw=-25)
    _ref(O + "/SM_A4_Table.usd", "/World/lr/coffee", pos=(-1.5, 0.0, 0))
    _ref(O + "/SM_Plant01.usd", "/World/lr/plant1", pos=(-3.3, 3.2, 0))
    _ref(O + "/SM_Plant02.usd", "/World/lr/plant2", pos=(-3.3, -3.2, 0))
    _ref(O + "/SM_Cupboard.usd", "/World/lr/cupboard", pos=(2.6, 3.2, 0), yaw=180)
    return {"spawn": (1.4, 0.0, 180.0), "look": (-1.2, 0.0, 0.8), "eye": (4.6, -3.4, 2.6)}


_SCENES = {"warehouse": load_warehouse, "kitchen": load_kitchen, "living_room": load_living_room}


def names():
    return list(_SCENES)


def load(name, world):
    """Build the named environment in ``world``; returns spawn + camera hints."""
    if name not in _SCENES:
        raise ValueError(f"unknown scene {name!r}; choose from {names()}")
    return _SCENES[name](world)
