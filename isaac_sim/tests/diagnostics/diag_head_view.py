"""Find a usable head-D435 RGB: detect self-occlusion, place a clear workspace.

1) Render the matched head cam with NO props at a few FORWARD offsets along its
   optical axis — if something sits at ~0.05 m (head housing in front of the
   lens), an offset clears it and tells us the camera should mount further fwd.
2) Place a table+cube FAR forward and LOW (where the 24deg-down view clears the
   robot body), light it (overhead work light + a front fill below head height),
   and render the matched 69deg cam -> a usable workspace RGB + depth.
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
SETTLE, RESETTLE, WARMUP = 150, 70, 20
TILT = -0.65   # clamps to the ~-0.45 joint limit
CUBE = 0.06


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd, carb
        from PIL import Image
        from pxr import UsdGeom, UsdLux, Sdf, Gf
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import VisualCuboid
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        gmin, _ = scene_mod.world_range(rpath); ground_z = float(gmin[2])
        head_prim = sensors_mod._ensure_camera_prim()
        stage = omni.usd.get_context().get_stage()
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot.go_home()
        for _ in range(SETTLE):
            world.step(render=True)
        robot.set_head(0.0, TILT)
        for _ in range(RESETTLE):
            world.step(render=True)
        cpos, cfwd = sensors_mod.camera_world_pose(head_prim)
        cpos = np.array(cpos); cfwd = np.array(cfwd)

        def matched_cam(offset):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(head_prim))
            src = UsdGeom.Camera(stage.GetPrimAtPath(head_prim))
            cam = UsdGeom.Camera.Define(stage, "/MatchedHeadCam")
            cam.GetFocalLengthAttr().Set(float(src.GetFocalLengthAttr().Get()))
            cam.GetHorizontalApertureAttr().Set(float(src.GetHorizontalApertureAttr().Get()))
            cam.GetVerticalApertureAttr().Set(float(src.GetVerticalApertureAttr().Get()))
            cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 100.0))
            m2 = Gf.Matrix4d(m)
            pos = cpos + offset * cfwd
            m2.SetTranslateOnly(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
            xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(m2)

        def grab(want_depth):
            rp = rep.create.render_product("/MatchedHeadCam", (640, 480))
            ra = rep.AnnotatorRegistry.get_annotator("rgb"); ra.attach(rp)
            da = rep.AnnotatorRegistry.get_annotator("distance_to_camera") if want_depth else None
            if da:
                da.attach(rp)
            for _ in range(WARMUP):
                world.step(render=True)
            rgb = ra.get_data(do_array_copy=True)
            dep = da.get_data(do_array_copy=True) if da else None
            ra.detach()
            if da:
                da.detach()
            rp.destroy()
            return rgb, dep

        def save_rgb(rgb, name):
            a = np.asarray(rgb); a = a[:, :, :3] if a.ndim == 3 and a.shape[2] == 4 else a
            Image.fromarray(a.astype(np.uint8), "RGB").save(OUT / name)
            print(f"[hv] wrote {name} mean={a.mean():.0f}", flush=True)

        def near_frac(dep, thr=0.4):
            d = np.asarray(dep, np.float32); d = d[:, :, 0] if d.ndim == 3 else d
            fin = d[np.isfinite(d)]
            mn = float(fin.min()) if fin.size else -1
            frac = float((fin < thr).sum()) / d.size if fin.size else 0
            return mn, frac

        # ---- (1) self-occlusion check: no props, forward offsets ----
        print(f"[hv] head pos={cpos.round(3)} fwd={cfwd.round(3)} (tilt clamped)", flush=True)
        for off in (0.0, 0.12, 0.25):
            matched_cam(off)
            rgb, dep = grab(True)
            mn, frac = near_frac(dep)
            print(f"[hv] self off={off:.2f}: nearest={mn:.3f} m, <0.4m={frac*100:.0f}% of frame", flush=True)
            save_rgb(rgb, f"cam_self_off{int(off*100):02d}.png")

        # ---- (2) workspace far-forward + low, clear of the body ----
        OFF = 0.20                      # clear the head housing
        matched_cam(OFF)
        epos = cpos + OFF * cfwd
        table_top = max(0.40, ground_z + 0.40)
        # put the cube where the optical axis pierces the table-top plane
        t = (table_top - epos[2]) / cfwd[2]
        look = epos + t * cfwd
        cube_c = np.array([look[0], look[1], table_top + CUBE / 2])
        bench_h = table_top - ground_z
        bench_c = np.array([cube_c[0], cube_c[1], ground_z + bench_h / 2])
        print(f"[hv] workspace: cube {cube_c.round(3)} table_top={table_top:.2f} "
              f"(dist along axis={t:.2f} m)", flush=True)
        VisualCuboid(prim_path="/Preview/ground", name="g",
                     position=np.array([cube_c[0]*0.5, cube_c[1]*0.5, ground_z-0.01]),
                     scale=np.array([8.0, 8.0, 0.02]), color=np.array([0.32, 0.36, 0.44]))
        VisualCuboid(prim_path="/Preview/bench", name="b", position=bench_c,
                     scale=np.array([0.40, 0.40, bench_h]), color=np.array([0.62, 0.46, 0.30]))
        VisualCuboid(prim_path="/Preview/cube", name="c", position=cube_c,
                     scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]))

        def sphere(path, pos, r=0.18):
            s = UsdLux.SphereLight.Define(stage, Sdf.Path(path))
            s.CreateIntensityAttr(0.0); s.CreateRadiusAttr(r)
            x = UsdGeom.Xformable(s.GetPrim()); x.ClearXformOpOrder()
            x.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        sphere("/Preview/WorkTop", (cube_c[0]-0.1, cube_c[1], cube_c[2]+0.6))
        sphere("/Preview/WorkFront", (cube_c[0]-0.45, cube_c[1], table_top+0.15))

        def setI(path, v):
            UsdLux.LightAPI(stage.GetPrimAtPath(path)).GetIntensityAttr().Set(float(v))

        def patches(rgb):
            x = np.asarray(rgb)[:, :, :3]
            h, w = x.shape[:2]
            cube = x[int(h*0.30):int(h*0.45), int(w*0.40):int(w*0.60)]   # cube top
            front = x[int(h*0.55):int(h*0.85), int(w*0.35):int(w*0.65)]  # table front
            return x.mean(), cube.mean(0).mean(0), front.mean()

        # Everything renders ~234 (washed) regardless of light intensity, so the
        # lever is EXPOSURE, not lights. Disable histogram auto-exposure AND
        # sweep the camera's manual exposure (stops, log2; negative darkens).
        st = carb.settings.get_settings()
        st.set("/rtx/post/histogram/enabled", False)
        st.set("/rtx/post/tonemap/cameraExposureEnabled", True)
        setI("/Preview/WorkTop", 20000); setI("/Preview/WorkFront", 60000)
        mcam = UsdGeom.Camera(stage.GetPrimAtPath("/MatchedHeadCam"))
        for expo, name in [(0.0, "A"), (-3.0, "B"), (-5.0, "C"), (-7.0, "D")]:
            mcam.CreateExposureAttr().Set(float(expo))
            st.set("/rtx/post/tonemap/cameraExposure", float(expo))
            rgb, _ = grab(False)
            ov, cube_rgb, fr = patches(rgb)
            save_rgb(rgb, f"cam_proto_{name}.png")
            print(f"[hv]   {name}(exposure={expo}): overall={ov:.0f} "
                  f"cube_top_rgb={np.round(cube_rgb).astype(int)} front={fr:.0f}", flush=True)

        # depth at config A intensities
        setI("/Preview/WorkTop", 15000); setI("/Preview/WorkFront", 30000)
        _, dep = grab(True)
        d = np.asarray(dep, np.float32); d = d[:, :, 0] if d.ndim == 3 else d
        fin = d[np.isfinite(d)]
        if fin.size:
            print(f"[hv] proto depth {fin.min():.3f}..{fin.max():.3f} m (valid {100*fin.size/d.size:.0f}%)", flush=True)
        cl = np.clip(np.nan_to_num(d, nan=2.5, posinf=2.5), 0.3, 2.5)
        norm = ((cl-0.3)/2.2*255).astype(np.uint8)
        try:
            import matplotlib.cm as cm
            Image.fromarray((cm.get_cmap("turbo")(norm/255.0)*255).astype(np.uint8)[:, :, :3], "RGB").save(OUT/"cam_proto_depth_color.png")
        except Exception:
            pass
        print("[hv] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
