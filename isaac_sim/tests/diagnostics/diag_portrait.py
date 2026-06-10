"""Clean, well-lit portrait of the head + upper body with the D435 box hidden,
to confirm the head camera reads right (no floating offset, no crude box)."""
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
        from pxr import UsdGeom, UsdLux, Gf, Sdf
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        scene_mod._hide_legacy_hand_flanges()
        for _ in range(150):
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()
        from pxr import UsdGeom as _UG, Usd as _U
        import numpy as _np
        _bc = _UG.BBoxCache(_U.TimeCode.Default(), [_UG.Tokens.default_, _UG.Tokens.render])
        for p in stage.Traverse():
            if p.GetTypeName() != "Mesh":
                continue
            r = _bc.ComputeWorldBound(p).ComputeAlignedRange()
            mn, mx = r.GetMin(), r.GetMax()
            cz = (mn[2] + mx[2]) / 2
            if cz > 0.86:   # head region
                ctr = [round((mn[i]+mx[i])/2, 3) for i in range(3)]
                sz = [round(mx[i]-mn[i], 3) for i in range(3)]
                print(f"[pt] head mesh {p.GetName():22s} vis={_UG.Imageable(p).ComputeVisibility():9s} ctr={ctr} size={sz}", flush=True)

        # dark backdrop behind the robot for contrast with the white body
        scene_mod.add_visual_box("/Back/wall", (-1.2, -0.3, 0.7), (0.1, 4.0, 3.0), (0.10, 0.11, 0.13))
        # even, bright lighting so the white head isn't black; soft fills all round
        for ln, inten in (("/DomeLight", 800.0), ("/KeyLight", 2500.0), ("/FillLight", 1800.0)):
            lp = stage.GetPrimAtPath(ln)
            if lp:
                UsdLux.LightAPI(lp).GetIntensityAttr().Set(inten)
        for i, (dx, dy, dz) in enumerate([(0.6, -0.4, 0.6), (0.5, 0.5, 0.4)]):
            sl = UsdLux.SphereLight.Define(stage, Sdf.Path(f"/Fl/l{i}"))
            sl.CreateIntensityAttr(30000.0); sl.CreateRadiusAttr(0.3)
            x = UsdGeom.Xformable(sl.GetPrim()); x.ClearXformOpOrder()
            x.AddTranslateOp().Set(Gf.Vec3d(dx, -0.26 + dy, 0.95 + dz))

        focus = np.array([0.0, -0.26, 0.86])
        for name, dvec, dist in [("head_portrait.png", np.array([0.7, -0.35, 0.32]), 1.15),
                                 ("head_portrait_side.png", np.array([0.45, -0.7, 0.30]), 1.1)]:
            dvec = dvec / np.linalg.norm(dvec)
            eye = focus + dist * dvec
            cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                               look_at=tuple(float(v) for v in focus))
            rp = rep.create.render_product(str(cam.GetPath()), (820, 900))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(16):
                world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:, :, :3].astype(np.uint8), "RGB").save(OUT / name)
            a.detach(); rp.destroy(); print(f"[pt] wrote {name}", flush=True)
        print("[pt] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
