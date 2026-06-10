"""Show BOTH head cameras color-coded to decide the right one:
  camera_link (upstream head camera)  -> RED
  head_camera_link/box (my added D435) -> GREEN
Prints their world positions + forward axes; renders head front + side."""
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
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf, Vt
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        for _ in range(150):
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()

        def col(prim, rgb):
            for p in [prim] + list(prim.GetChildren()):
                if p.GetTypeName() == "Mesh":
                    m = UsdGeom.Mesh(p)
                    UsdGeom.Imageable(p).MakeVisible()
                    m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))

        def find(n, typ="Xform"):
            return next((p for p in stage.Traverse()
                         if p.GetName() == n and p.GetTypeName() == typ), None)

        cl = find("camera_link")
        if cl:
            UsdGeom.Imageable(cl).MakeVisible()   # the real upstream D435 (native material)
        box = stage.GetPrimAtPath(
            "/r2d3_v1/Geometry/base_link_underpan/body_base_link/platform_base_link/"
            "head_link1/head_link2/head_camera_bottom_screw_frame/head_camera_link/box")
        if box:
            UsdGeom.Imageable(box).MakeInvisible()   # hide my redundant added D435

        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        def info(name, prim):
            xf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            o = xf.Transform(Gf.Vec3d(0, 0, 0))
            fx = np.array(xf.Transform(Gf.Vec3d(1, 0, 0))) - np.array([o[0], o[1], o[2]])
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            print(f"[hc2] {name:16s} pos={np.array([o[0],o[1],o[2]]).round(3)} +Xaxis={fx.round(2)} "
                  f"bbox={[round(r.GetMin()[i],3) for i in range(3)]}..{[round(r.GetMax()[i],3) for i in range(3)]}", flush=True)
        h2 = find("head_link2"); info("head_link2", h2)
        if cl: info("camera_link", cl)
        hcl = find("head_camera_link");
        if hcl: info("head_camera_link", hcl)

        scene_mod.add_visual_box("/Back/wall", (-1.2, -0.3, 0.7), (0.1, 4.0, 3.0), (0.10, 0.11, 0.13))
        for ln, inten in (("/DomeLight", 700.0), ("/KeyLight", 2500.0), ("/FillLight", 1800.0)):
            lp = stage.GetPrimAtPath(ln)
            if lp: UsdLux.LightAPI(lp).GetIntensityAttr().Set(inten)
        for i,(dx,dy,dz) in enumerate([(0.5,-0.4,0.5),(0.4,0.4,0.4)]):
            sl=UsdLux.SphereLight.Define(stage, Sdf.Path(f"/Fl/l{i}")); sl.CreateIntensityAttr(30000.0); sl.CreateRadiusAttr(0.3)
            x=UsdGeom.Xformable(sl.GetPrim()); x.ClearXformOpOrder(); x.AddTranslateOp().Set(Gf.Vec3d(dx,-0.26+dy,0.97+dz))

        focus=np.array([0.0,-0.26,0.95])
        for name,dvec,dist in [("hc2_front.png",np.array([0.7,-0.30,0.42]),1.05),
                               ("hc2_side.png", np.array([0.40,-0.70,0.42]),1.05)]:
            dvec=dvec/np.linalg.norm(dvec); eye=focus+dist*dvec
            cam=rep.functional.create.camera(position=tuple(float(v) for v in eye),look_at=tuple(float(v) for v in focus))
            rp=rep.create.render_product(str(cam.GetPath()),(900,760))
            a=rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(16): world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:,:,:3].astype(np.uint8),"RGB").save(OUT/name)
            a.detach(); rp.destroy(); print(f"[hc2] wrote {name}", flush=True)
        print("[hc2] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
