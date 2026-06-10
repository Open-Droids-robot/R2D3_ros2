"""Camera-match test + lighting sweep for the head D435 down-view.

Two jobs in one Isaac boot:

  TEST (camera match): build /MatchedHeadCam — a root-level UsdGeom.Camera whose
  world transform is COPIED from the mounted head camera prim (so position,
  angle, AND sense/up all match) and whose focal/aperture match (69 deg HFOV).
  Print the head prim basis vs the matched cam basis and the FOVs; they must
  agree. (We render the head view through this matched cam because the
  articulation-child prim renders from its rest pose in the offline path.)

  SWEEP (lighting): render the head-down workspace through the matched cam under
  several lighting/exposure configs, so we can pick the one that lights the
  props' camera-facing faces (which were rendering pure black).
"""
from __future__ import annotations
import os, sys, math
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

OUT = _REPO / "isaac_sim/tests/captures"
LOOK_DIST_M = 0.95
CUBE = 0.06
TABLE_W = 0.30
SETTLE = 150
RESETTLE = 70
WARMUP = 18
HEAD_TILT_DOWN = -0.45
MATCHED = "/MatchedHeadCam"


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
        robot_min0, _ = scene_mod.world_range(rpath)
        ground_z = float(robot_min0[2])
        head_prim = sensors_mod._ensure_camera_prim()
        stage = omni.usd.get_context().get_stage()
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        robot.go_home()
        for _ in range(SETTLE):
            world.step(render=True)
        robot.set_head(0.0, HEAD_TILT_DOWN)
        for _ in range(RESETTLE):
            world.step(render=True)

        def basis(prim_path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
            o = np.array(m.Transform(Gf.Vec3d(0, 0, 0)))
            f = np.array(m.Transform(Gf.Vec3d(0, 0, -1))) - o
            u = np.array(m.Transform(Gf.Vec3d(0, 1, 0))) - o
            r = np.array(m.Transform(Gf.Vec3d(1, 0, 0))) - o
            return o, f / np.linalg.norm(f), u / np.linalg.norm(u), r / np.linalg.norm(r), m

        def fov(prim_path):
            c = UsdGeom.Camera(stage.GetPrimAtPath(prim_path))
            fl = float(c.GetFocalLengthAttr().Get()); ha = float(c.GetHorizontalApertureAttr().Get())
            va = float(c.GetVerticalApertureAttr().Get())
            return (math.degrees(2 * math.atan(ha / (2 * fl))),
                    math.degrees(2 * math.atan(va / (2 * fl))))

        # ---- build matched cam: copy head prim world transform + intrinsics ----
        ho, hf, hu, hr, hM = basis(head_prim)
        mc = UsdGeom.Camera.Define(stage, MATCHED)
        src = UsdGeom.Camera(stage.GetPrimAtPath(head_prim))
        mc.GetFocalLengthAttr().Set(float(src.GetFocalLengthAttr().Get()))
        mc.GetHorizontalApertureAttr().Set(float(src.GetHorizontalApertureAttr().Get()))
        mc.GetVerticalApertureAttr().Set(float(src.GetVerticalApertureAttr().Get()))
        mc.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
        mx = UsdGeom.Xformable(mc); mx.ClearXformOpOrder()
        mx.AddTransformOp().Set(hM)               # root-level: local == world
        mo, mf, mu, mr, _ = basis(MATCHED)

        hfov_h, vfov_h = fov(head_prim); hfov_m, vfov_m = fov(MATCHED)
        print("[match] ===== CAMERA MATCH TEST =====", flush=True)
        print(f"[match] head  pos={ho.round(4)} fwd={hf.round(4)} up={hu.round(4)} right={hr.round(4)}", flush=True)
        print(f"[match] match pos={mo.round(4)} fwd={mf.round(4)} up={mu.round(4)} right={mr.round(4)}", flush=True)
        dpos = float(np.linalg.norm(ho - mo)); dfwd = float(np.linalg.norm(hf - mf))
        dup = float(np.linalg.norm(hu - mu)); drt = float(np.linalg.norm(hr - mr))
        print(f"[match] delta pos={dpos:.2e} fwd={dfwd:.2e} up={dup:.2e} right={drt:.2e}", flush=True)
        print(f"[match] head FOV={hfov_h:.2f}x{vfov_h:.2f}  matched FOV={hfov_m:.2f}x{vfov_m:.2f}", flush=True)
        ok = max(dpos, dfwd, dup, drt) < 1e-4 and abs(hfov_h - hfov_m) < 1e-3
        print(f"[match] MATCH {'PASS' if ok else 'FAIL'} (tol 1e-4, FOV 1e-3)", flush=True)

        # ---- workspace props (VisualCuboid: proper materials) ----
        cube_c = ho + LOOK_DIST_M * hf
        top_z = float(cube_c[2] - CUBE / 2); bench_h = max(0.1, top_z - ground_z)
        bench_c = np.array([cube_c[0], cube_c[1], ground_z + bench_h / 2])
        VisualCuboid(prim_path="/Preview/ground", name="pg",
                     position=np.array([cube_c[0]*0.5, cube_c[1]*0.5, ground_z-0.01]),
                     scale=np.array([6.0, 6.0, 0.02]), color=np.array([0.30, 0.34, 0.42]))
        VisualCuboid(prim_path="/Preview/workbench", name="pb", position=bench_c,
                     scale=np.array([TABLE_W, TABLE_W, bench_h]), color=np.array([0.62, 0.46, 0.30]))
        VisualCuboid(prim_path="/Preview/cube_red", name="pc", position=cube_c,
                     scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]))

        def grab():
            rp = rep.create.render_product(MATCHED, (640, 480))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(WARMUP):
                world.step(render=True)
            data = np.asarray(a.get_data(do_array_copy=True))
            a.detach(); rp.destroy()
            return data

        def save(rgb, name):
            x = rgb[:, :, :3].astype(np.uint8) if rgb.ndim == 3 and rgb.shape[2] == 4 else rgb.astype(np.uint8)
            Image.fromarray(x, "RGB").save(OUT / name)
            h, w = x.shape[:2]
            front = x[int(h*0.6):int(h*0.9), int(w*0.35):int(w*0.65)]   # table FRONT face
            bg = x[0:int(h*0.15), 0:int(w*0.2)]                          # top-left background
            print(f"[sweep] {name:34s} overall={x.mean():.0f} front={front.mean():.0f} "
                  f"bg={bg.mean():.0f}  (want front 110-190, bg <190)", flush=True)

        # quaternion mapping local -Z -> dir (to aim a distant light along a dir)
        def aim_neg_z(prim, dir_world):
            d = np.asarray(dir_world, float); d /= np.linalg.norm(d)
            rot = Gf.Rotation(Gf.Vec3d(0, 0, -1), Gf.Vec3d(*d))
            q = rot.GetQuat()
            xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
            xf.AddOrientOp().Set(Gf.Quatf(q))

        settings = carb.settings.get_settings()

        def set_intensity(path, val):
            p = stage.GetPrimAtPath(path)
            if p:
                UsdLux.LightAPI(p).GetIntensityAttr().Set(val)

        # ---- SWEEP: dome intensity (omnidirectional, lights all faces) ----
        # front-face patch mean wants ~100-180 (lit, colored, not blown out);
        # overall mean shouldn't saturate (>235 = washed out).
        print("[sweep] ===== LIGHTING SWEEP: dark bg dome + workspace key light =====", flush=True)
        # Workspace key light ABOVE-FRONT of the table (like a desk lamp) — lights
        # the table top, cube, and camera-facing front faces without blasting the
        # robot's near body (the headlight-at-camera mistake) and a DARKER dome so
        # the bright sky doesn't force auto-exposure to crush the props.
        wl = UsdLux.SphereLight.Define(stage, Sdf.Path("/Preview/WorkLight"))
        wl.CreateRadiusAttr(0.12); wl.CreateIntensityAttr(0.0)
        wlx = UsdGeom.Xformable(wl.GetPrim()); wlx.ClearXformOpOrder()
        wlx.AddTranslateOp().Set(Gf.Vec3d(float(cube_c[0]) - 0.30, float(cube_c[1]),
                                          float(cube_c[2]) + 0.40))

        for dome_i, key_i, fill_i, work_i, name in [
            (300,  2500, 1500, 0,      "cam_light_A_darkdome.png"),
            (300,  2500, 1500, 8.0e4,  "cam_light_B_work80k.png"),
            (300,  2500, 1500, 2.0e5,  "cam_light_C_work200k.png"),
            (600,  2500, 1500, 1.5e5,  "cam_light_D_dome600_work150k.png"),
            (150,  1500, 800,  1.2e5,  "cam_light_E_verydark_work120k.png"),
        ]:
            set_intensity("/DomeLight", float(dome_i))
            set_intensity("/KeyLight", float(key_i))
            set_intensity("/FillLight", float(fill_i))
            set_intensity("/Preview/WorkLight", float(work_i))
            save(grab(), name)

        print("[sweep] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
