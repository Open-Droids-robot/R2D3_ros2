"""Render-free: what occludes the head D435, and how far forward to clear it.

Prints the world AABB of head_link2 / head_camera_link and their child visual
prims, plus the camera world position + forward, so we can compute the mount
offset needed to push the lens past the head shroud.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
try:
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, Gf
    from isaacsim.core.api import World
    from isaac_sim.r2d3_sim import scene as scene_mod
    from isaac_sim.r2d3_sim import sensors as sensors_mod
    from isaac_sim.r2d3_sim.robot import Robot

    world = World(stage_units_in_meters=1.0)
    rpath = scene_mod.assemble(world)
    head_prim = sensors_mod._ensure_camera_prim()
    world.reset()
    robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
    for _ in range(120):
        world.step(render=False)
    robot.set_head(0.0, 0.0)   # LEVEL: camera forward = head_link2 +X
    for _ in range(60):
        world.step(render=False)

    stage = omni.usd.get_context().get_stage()
    cpos, cfwd = sensors_mod.camera_world_pose(head_prim)
    cpos = np.array(cpos); cfwd = np.array(cfwd)
    print(f"[occ] camera pos={cpos.round(3)} fwd(level)={cfwd.round(3)}", flush=True)

    HEAD2 = sensors_mod.HEAD_CAMERA_LINK.rsplit("/", 1)[0]  # parent of head_camera_link chain
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    def report(path):
        p = stage.GetPrimAtPath(path)
        if not p:
            print(f"[occ] (missing) {path}", flush=True); return
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        # how far does it extend FORWARD (along +cfwd) past the camera?
        fwd_extent = float(np.dot(np.array([mx[0], cpos[1], cpos[2]]) - cpos, cfwd))
        print(f"[occ] {path}", flush=True)
        print(f"[occ]    x[{mn[0]:.3f},{mx[0]:.3f}] y[{mn[1]:.3f},{mx[1]:.3f}] "
              f"z[{mn[2]:.3f},{mx[2]:.3f}]  +X past cam={mx[0]-cpos[0]:.3f} m", flush=True)

    print(f"[occ] head link subtree = {HEAD2}", flush=True)
    report(HEAD2)
    report(sensors_mod.HEAD_CAMERA_LINK)
    # list direct children of head_camera_link's parent chain
    for base in (HEAD2, sensors_mod.HEAD_CAMERA_LINK):
        bp = stage.GetPrimAtPath(base)
        if not bp:
            continue
        for ch in bp.GetChildren():
            cp = ch.GetPath().pathString
            if UsdGeom.Imageable(ch):
                try:
                    rr = cache.ComputeWorldBound(ch).ComputeAlignedRange()
                    mn, mx = rr.GetMin(), rr.GetMax()
                    if not rr.IsEmpty():
                        print(f"[occ]   child {cp}: x[{mn[0]:.3f},{mx[0]:.3f}] "
                              f"+X past cam={mx[0]-cpos[0]:.3f}", flush=True)
                except Exception:
                    pass
    print("[occ] DONE", flush=True)
finally:
    app.close()
