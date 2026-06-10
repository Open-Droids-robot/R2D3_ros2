"""Capture what EACH R2D3 camera sees, in one Isaac boot.

The robot has one physical camera: the head Intel D435 (RGB + depth).

  cam_overview.png         third-person context (synthetic free cam)
  cam_head_down_rgb.png    head D435 looking down at the workspace
  cam_head_down_depth*.png head D435 depth, same view
  cam_head_level_rgb.png   head D435 looking straight ahead (tilt 0)

The head views are rendered through a MATCHED camera: a root-level UsdGeom.Camera
whose world transform is COPIED from the mounted head-camera prim (so position,
angle and sense all match) and whose intrinsics match (69 deg RGB FOV). We use
this instead of the mounted prim because the articulation-child prim renders
from its rest pose in the offline replicator path; the matched cam reproduces
the posed head exactly (verified by diag_camera_light.py -> MATCH PASS).

Lighting: the head tilts down enough to look at the table TOP (lit by the
overhead key/fill) rather than its camera-facing front face (which no overhead
light reaches -> renders black). An overhead work light lifts the tabletop+cube.
"""
from __future__ import annotations
import argparse, os, sys, math
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

LOOK_DIST_M = 0.80
CUBE = 0.06
TABLE_W = 0.34
SETTLE = 150
RESETTLE = 70
WARMUP = 20
HEAD_TILT_DOWN = -0.65       # steeper: look at the lit tabletop, not the dark front face
MATCHED = "/MatchedHeadCam"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=_REPO / "isaac_sim/tests/captures")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
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
        print(f"[cam] settled home: lift={robot.get_lift_m():.3f} m", flush=True)

        # ---- matched camera: exact copy of the head prim transform + FOV ----
        def sync_matched():
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(head_prim))
            src = UsdGeom.Camera(stage.GetPrimAtPath(head_prim))
            cam = UsdGeom.Camera.Define(stage, MATCHED)
            cam.GetFocalLengthAttr().Set(float(src.GetFocalLengthAttr().Get()))
            cam.GetHorizontalApertureAttr().Set(float(src.GetHorizontalApertureAttr().Get()))
            cam.GetVerticalApertureAttr().Set(float(src.GetVerticalApertureAttr().Get()))
            cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
            xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(m)
            return cam

        def cam_pose():
            return sensors_mod.camera_world_pose(head_prim)

        def grab(camera_path, want_depth):
            rp = rep.create.render_product(camera_path, (640, 480))
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

        def save_rgb(rgb, path):
            a = np.asarray(rgb)
            a = a[:, :, :3] if a.ndim == 3 and a.shape[2] == 4 else a
            a = a.astype(np.uint8)
            Image.fromarray(a, "RGB").save(path)
            print(f"[cam] wrote {path.name}  {a.shape} mean={a.mean():.0f}", flush=True)

        def save_depth(dep, gray, color, lo=0.3, hi=1.8):
            d = np.asarray(dep, dtype=np.float32)
            d = d[:, :, 0] if d.ndim == 3 else d
            fin = d[np.isfinite(d)]
            if fin.size:
                print(f"[cam] depth range {fin.min():.3f}..{fin.max():.3f} m "
                      f"(valid {100*fin.size/d.size:.0f}%)", flush=True)
            cl = np.clip(np.nan_to_num(d, nan=hi, posinf=hi), lo, hi)
            norm = ((cl - lo) / (hi - lo) * 255).astype(np.uint8)
            Image.fromarray(norm, "L").save(gray)
            try:
                import matplotlib.cm as cm
                rgba = (cm.get_cmap("turbo")(norm / 255.0) * 255).astype(np.uint8)
                Image.fromarray(rgba[:, :, :3], "RGB").save(color)
            except Exception:  # noqa: BLE001
                pass

        # ---- tilt head down, place workspace on the optical axis ----
        robot.set_head(0.0, HEAD_TILT_DOWN)
        for _ in range(RESETTLE):
            world.step(render=True)
        cpos, cfwd = cam_pose()
        cube_c = np.array(cpos) + LOOK_DIST_M * np.array(cfwd)
        top_z = float(cube_c[2] - CUBE / 2); bench_h = max(0.1, top_z - ground_z)
        bench_c = np.array([cube_c[0], cube_c[1], ground_z + bench_h / 2])
        print(f"[cam] head(down) pos={np.array(cpos).round(3)} fwd={np.array(cfwd).round(3)} "
              f"-> cube {cube_c.round(3)} table_top={top_z:.3f}", flush=True)
        VisualCuboid(prim_path="/Preview/ground", name="pg",
                     position=np.array([cube_c[0]*0.5, cube_c[1]*0.5, ground_z-0.01]),
                     scale=np.array([6.0, 6.0, 0.02]), color=np.array([0.32, 0.36, 0.44]))
        VisualCuboid(prim_path="/Preview/workbench", name="pb", position=bench_c,
                     scale=np.array([TABLE_W, TABLE_W, bench_h]), color=np.array([0.62, 0.46, 0.30]))
        VisualCuboid(prim_path="/Preview/cube_red", name="pc", position=cube_c,
                     scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]))
        # Overhead work light: lifts the tabletop + cube top (the faces the
        # steep down-view sees), without blasting the robot's near body.
        wl = UsdLux.SphereLight.Define(stage, Sdf.Path("/Preview/WorkLight"))
        wl.CreateIntensityAttr(120000.0); wl.CreateRadiusAttr(0.18)
        wlx = UsdGeom.Xformable(wl.GetPrim()); wlx.ClearXformOpOrder()
        wlx.AddTranslateOp().Set(Gf.Vec3d(float(cube_c[0]) - 0.1, float(cube_c[1]), float(cube_c[2]) + 0.5))

        # ---- overview ----
        rmin, rmax = scene_mod.world_range(rpath)
        u_min = np.minimum.reduce([rmin, cube_c - 0.3, bench_c - 0.3])
        u_max = np.maximum.reduce([rmax, cube_c + 0.3, bench_c + 0.3])
        center = (u_min + u_max) / 2
        radius = 0.5 * float(np.linalg.norm(u_max - u_min))
        dd = np.array([0.62, -0.72, 0.31]); dd /= np.linalg.norm(dd)
        eye = center + max(3.3, 2.3 * radius) * dd
        ov = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                          look_at=tuple(float(v) for v in center))
        print("[cam] rendering overview...", flush=True)
        rgb, _ = grab(str(ov.GetPath()), False)
        save_rgb(rgb, args.out / "cam_overview.png")

        # ---- head DOWN via matched cam (RGB + depth) ----
        print("[cam] rendering head D435 (down) via matched cam...", flush=True)
        sync_matched()
        rgb, dep = grab(MATCHED, True)
        save_rgb(rgb, args.out / "cam_head_down_rgb.png")
        save_depth(dep, args.out / "cam_head_down_depth.png",
                   args.out / "cam_head_down_depth_color.png")

        # ---- head LEVEL via matched cam ----
        robot.set_head(0.0, 0.0)
        for _ in range(RESETTLE):
            world.step(render=True)
        cp, cf = cam_pose()
        print(f"[cam] head(level) pos={np.array(cp).round(3)} fwd={np.array(cf).round(3)}", flush=True)
        sync_matched()
        rgb, _ = grab(MATCHED, False)
        save_rgb(rgb, args.out / "cam_head_level_rgb.png")

        print("[cam] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
