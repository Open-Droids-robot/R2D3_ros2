"""Verify the 3-camera setup: head (camera_link) + 2 wrist (l/r_wrist_camera).
Prints each camera's world pos + the optical look direction, checks the wrist
cams aim toward their hands, and renders the robot + a wrist close-up."""
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
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        for _ in range(150):
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()

        def find(n):
            return next((p for p in stage.Traverse() if p.GetName() == n), None)
        def wp(prim):
            o = UsdGeom.XformCache().GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(0, 0, 0))
            return np.array([o[0], o[1], o[2]])
        def axis(prim, v):
            xf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            o = np.array(xf.Transform(Gf.Vec3d(0, 0, 0)))
            return np.array(xf.Transform(Gf.Vec3d(*v))) - o

        for camn, opt, handn in [("camera_link", "head_camera_color_optical_frame", None),
                                  ("l_wrist_camera_link", "l_wrist_camera_color_optical_frame", "l_hand_link"),
                                  ("r_wrist_camera_link", "r_wrist_camera_color_optical_frame", "r_hand")]:
            c = find(camn); of = find(opt)
            if not c:
                print(f"[cam] {camn} MISSING", flush=True); continue
            pos = wp(c)
            look = axis(of, (0, 0, 1)) if of else axis(c, (1, 0, 0))   # optical +Z is the view dir
            look = look / (np.linalg.norm(look) + 1e-9)
            msg = f"[cam] {camn:20s} pos={pos.round(3)} look={look.round(2)}"
            if handn:
                h = find(handn)
                if h:
                    to = wp(h) - pos; to = to / (np.linalg.norm(to) + 1e-9)
                    msg += f"  hand_dir={to.round(2)} dot={float(np.dot(look,to)):.2f} (1=aimed at hand)"
            print(msg, flush=True)

        # dark backdrop + light, render robot + left-wrist close-up
        scene_mod.add_visual_box("/Back/wall", (-1.3, -0.3, 0.8), (0.1, 4.0, 3.5), (0.10, 0.11, 0.13))
        for ln, inten in (("/DomeLight", 500.0), ("/KeyLight", 2600.0), ("/FillLight", 1800.0)):
            lp = stage.GetPrimAtPath(ln)
            if lp: UsdLux.LightAPI(lp).GetIntensityAttr().Set(inten)
        lwc = wp(find("l_wrist_camera_link"))

        def shot(name, focus, dvec, dist):
            dvec = np.array(dvec, float); dvec /= np.linalg.norm(dvec)
            eye = np.array(focus) + dist * dvec
            cam = rep.functional.create.camera(position=tuple(float(v) for v in eye), look_at=tuple(float(v) for v in focus))
            rp = rep.create.render_product(str(cam.GetPath()), (900, 760))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(16): world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:, :, :3].astype(np.uint8), "RGB").save(OUT / name)
            a.detach(); rp.destroy(); print(f"[cam] wrote {name}", flush=True)

        shot("cams_robot.png", [0.0, -0.25, 0.85], [0.7, -0.35, 0.35], 1.9)
        shot("cams_lwrist.png", list(lwc), [0.6, -0.5, 0.45], 0.5)
        print("[cam] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
