"""Clear close-up of the head + D435 camera body, to fix its positioning.

Poses the arms down (clear of the head), lights the head well, and renders a
tight front-3/4 view. Does NOT hide the D435 body box, so we see the camera
geometry on the head. Prints the camera + head_link2 geometry.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

OUT = _REPO / "isaac_sim/tests/captures"


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import Usd, UsdGeom, Gf, UsdLux, Sdf
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        scene_mod._hide_legacy_hand_flanges()   # also hides the D435 body box now
        # swing both arms down so they don't occlude the head
        for _ in range(200):
            robot.set_arm_targets("left", [0.0, -1.55, 0.0, 0.2, 0.0, 0.0, 0.0])
            robot.set_arm_targets("right", [0.0, -1.55, 0.0, 0.2, 0.0, 0.0, 0.0])
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()

        def find(n):
            return next((p.GetPath().pathString for p in stage.Traverse()
                         if p.GetName() == n and p.GetTypeName() == "Xform"), None)

        def wbbox(path):
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            r = bc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            return np.array([r.GetMin()[i] for i in range(3)]), np.array([r.GetMax()[i] for i in range(3)])

        h2 = find("head_link2"); box = stage.GetPrimAtPath(
            "/r2d3_v1/Geometry/base_link_underpan/body_base_link/platform_base_link/"
            "head_link1/head_link2/head_camera_bottom_screw_frame/head_camera_link/box")
        h2_min, h2_max = wbbox(h2)
        hc = (h2_min + h2_max) / 2
        print(f"[hc] head_link2 bbox {h2_min.round(3)}..{h2_max.round(3)} center {hc.round(3)}", flush=True)
        print(f"[hc] D435 'box' prim valid={bool(box)} visible="
              f"{UsdGeom.Imageable(box).ComputeVisibility() if box else 'n/a'}", flush=True)
        if box:
            bmin, bmax = wbbox(box.GetPath().pathString)
            print(f"[hc] D435 box bbox {bmin.round(3)}..{bmax.round(3)} size {(bmax-bmin).round(3)}", flush=True)

        # DARK background so the white head stands out (white-on-white otherwise)
        for ln, inten in (("/DomeLight", 25.0), ("/KeyLight", 3500.0), ("/FillLight", 1200.0)):
            lp = stage.GetPrimAtPath(ln)
            if lp:
                UsdLux.LightAPI(lp).GetIntensityAttr().Set(inten)
        sl = UsdLux.SphereLight.Define(stage, Sdf.Path("/Marker/light"))
        sl.CreateIntensityAttr(50000.0); sl.CreateRadiusAttr(0.2)
        slx = UsdGeom.Xformable(sl.GetPrim()); slx.ClearXformOpOrder()
        slx.AddTranslateOp().Set(Gf.Vec3d(float(hc[0])+0.4, float(hc[1])-0.3, float(hc[2])+0.3))

        for name, dvec, dist in [("headcam_front.png", np.array([0.6, -0.25, 0.45]), 0.9),
                                 ("headcam_3q.png",    np.array([0.55, -0.55, 0.5]), 0.9)]:
            dvec = dvec / np.linalg.norm(dvec)
            eye = hc + dist * dvec
            cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                               look_at=tuple(float(v) for v in hc))
            rp = rep.create.render_product(str(cam.GetPath()), (900, 760))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(14):
                world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:, :, :3].astype(np.uint8), "RGB").save(OUT / name)
            a.detach(); rp.destroy(); print(f"[hc] wrote {name}", flush=True)
        print("[hc] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
