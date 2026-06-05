"""Capture what EACH R2D3 camera sees, in one Isaac boot.

The robot has one physical camera: the head Intel D435 (RGB + depth). We
render, against a table+cube workspace, with the head tilted down to look at
it (the now-working tilt):

  cam_overview.png            third-person context (synthetic free cam)
  cam_head_down_realprim.png  head D435 looking down — through the ACTUAL
                              mounted camera prim (true 87.5° FOV). Warmed with
                              world.step (timeline PLAYING) so the mounted
                              camera tracks the posed head, instead of
                              rep.orchestrator.step(pause_timeline=True) which
                              freezes it at the rest pose.
  cam_head_down_freecam.png   same view via a free cam at the measured head
                              pose (default ~47° FOV) — narrower, for comparison
  cam_head_down_depth(_color).png   head D435 depth, looking down
  cam_head_level_rgb.png      head D435 looking straight ahead (tilt 0)

A local sphere light over the workspace + a boosted dome keep prop faces from
rendering black.
"""
from __future__ import annotations
import argparse, os, sys, math
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

LOOK_DIST_M = 0.95
CUBE = 0.06
TABLE_W = 0.30
SETTLE = 150
RESETTLE = 70
WARMUP = 20
HEAD_TILT_DOWN = -0.45


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
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        robot_min0, _ = scene_mod.world_range(rpath)
        ground_z = float(robot_min0[2])
        head_cam_prim = sensors_mod._ensure_camera_prim()
        stage = omni.usd.get_context().get_stage()
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        robot.go_home()
        for _ in range(SETTLE):
            world.step(render=True)
        print(f"[cam] settled home: lift={robot.get_lift_m():.3f} m", flush=True)

        gcam = UsdGeom.Camera(stage.GetPrimAtPath(head_cam_prim))
        fl = float(gcam.GetFocalLengthAttr().Get()); ha = float(gcam.GetHorizontalApertureAttr().Get())
        hfov = math.degrees(2 * math.atan(ha / (2 * fl)))
        print(f"[cam] head D435 prim true HFOV={hfov:.1f} deg", flush=True)

        # ---- helpers: warm with world.step so the timeline keeps PLAYING ----
        def grab(camera_path, res, want_depth):
            rp = rep.create.render_product(camera_path, res)
            rgb_a = rep.AnnotatorRegistry.get_annotator("rgb"); rgb_a.attach(rp)
            dep_a = None
            if want_depth:
                dep_a = rep.AnnotatorRegistry.get_annotator("distance_to_camera"); dep_a.attach(rp)
            for _ in range(WARMUP):
                world.step(render=True)
            rgb = rgb_a.get_data(do_array_copy=True)
            dep = dep_a.get_data(do_array_copy=True) if dep_a else None
            rgb_a.detach()
            if dep_a:
                dep_a.detach()
            rp.destroy()
            return rgb, dep

        def save_rgb(rgb, path):
            a = np.asarray(rgb)
            if a.ndim == 3 and a.shape[2] == 4:
                a = a[:, :, :3]
            a = a.astype(np.uint8)
            Image.fromarray(a, "RGB").save(path)
            print(f"[cam] wrote {path.name}  {a.shape} min={a.min()} max={a.max()} "
                  f"mean={a.mean():.0f}", flush=True)

        def save_depth(dep, gray, color, lo=0.3, hi=2.0):
            d = np.asarray(dep, dtype=np.float32)
            if d.ndim == 3:
                d = d[:, :, 0]
            fin = d[np.isfinite(d)]
            if fin.size:
                print(f"[cam] depth range: {fin.min():.3f}..{fin.max():.3f} m "
                      f"(valid {100*fin.size/d.size:.0f}%)", flush=True)
            cl = np.clip(np.nan_to_num(d, nan=hi, posinf=hi), lo, hi)
            norm = ((cl - lo) / (hi - lo) * 255).astype(np.uint8)
            Image.fromarray(norm, "L").save(gray)
            try:
                import matplotlib.cm as cm
                rgba = (cm.get_cmap("turbo")(norm / 255.0) * 255).astype(np.uint8)
                Image.fromarray(rgba[:, :, :3], "RGB").save(color)
                print(f"[cam] wrote {color.name}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[cam] colorize skipped: {e}", flush=True)

        # ---- tilt head DOWN, place workspace on its optical axis ----
        robot.set_head(0.0, HEAD_TILT_DOWN)
        for _ in range(RESETTLE):
            world.step(render=True)
        cam_pos, cam_fwd = sensors_mod.camera_world_pose(head_cam_prim)
        cube_c = np.array(cam_pos) + LOOK_DIST_M * np.array(cam_fwd)
        top_z = float(cube_c[2] - CUBE / 2)
        bench_h = max(0.1, top_z - ground_z)
        bench_c = (float(cube_c[0]), float(cube_c[1]), ground_z + bench_h / 2)
        print(f"[cam] head(down) pos={np.array(cam_pos).round(3)} fwd={np.array(cam_fwd).round(3)} "
              f"-> cube {cube_c.round(3)} table_top={top_z:.3f}", flush=True)
        # Use Isaac VisualCuboid (proper PBR visual material) instead of raw
        # displayColor cubes — the latter render camera-facing faces black
        # under RTX because the UsdPreviewSurface binding doesn't take. The
        # robot renders fine under the scene's dome+key+fill, so well-materialed
        # props will too (no extra lights needed).
        from isaacsim.core.api.objects import VisualCuboid
        VisualCuboid(prim_path="/Preview/ground", name="prev_ground",
                     position=np.array([cube_c[0] * 0.5, cube_c[1] * 0.5, ground_z - 0.01]),
                     scale=np.array([6.0, 6.0, 0.02]), color=np.array([0.30, 0.34, 0.42]))
        VisualCuboid(prim_path="/Preview/workbench", name="prev_bench",
                     position=np.array([bench_c[0], bench_c[1], bench_c[2]]),
                     scale=np.array([TABLE_W, TABLE_W, bench_h]),
                     color=np.array([0.62, 0.46, 0.30]))
        VisualCuboid(prim_path="/Preview/cube_red", name="prev_cube",
                     position=np.array([cube_c[0], cube_c[1], cube_c[2]]),
                     scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]))
        # Fill light aimed along +X and slightly down (comes from the robot/
        # camera side) so the props' camera-facing FRONT faces are lit — the
        # scene's key/fill come from above, leaving the head-down view's faces
        # black. RotateY(-70) maps the light's -Z emission to ~(+0.94,0,-0.34).
        ffill = UsdLux.DistantLight.Define(stage, Sdf.Path("/Preview/FrontFill"))
        ffill.CreateIntensityAttr(2800.0); ffill.CreateAngleAttr(2.0)
        ffx = UsdGeom.Xformable(ffill.GetPrim()); ffx.ClearXformOpOrder()
        ffx.AddRotateXYZOp().Set(Gf.Vec3f(0.0, -70.0, 0.0))

        # ---- overview ----
        rmin, rmax = scene_mod.world_range(rpath)
        u_min = np.minimum.reduce([rmin, cube_c - 0.3, np.array(bench_c) - 0.3])
        u_max = np.maximum.reduce([rmax, cube_c + 0.3, np.array(bench_c) + 0.3])
        center = (u_min + u_max) / 2
        radius = 0.5 * float(np.linalg.norm(u_max - u_min))
        d = np.array([0.62, -0.72, 0.31]); d /= np.linalg.norm(d)
        eye = center + max(3.3, 2.3 * radius) * d
        ov = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                          look_at=tuple(float(v) for v in center))
        print("[cam] rendering overview...", flush=True)
        rgb, _ = grab(str(ov.GetPath()), (1280, 720), False)
        save_rgb(rgb, args.out / "cam_overview.png")

        # ---- head DOWN via free cam at measured pose (RGB + depth) ----
        # The free cam reproduces the head camera's measured world pose +
        # orientation. (The actual mounted-camera prim render stays at the rest
        # pose in the offline replicator path — it does NOT track physics —
        # which is why we use the free cam here.)
        print("[cam] rendering head D435 (down) via free cam at measured pose...", flush=True)
        fc = rep.functional.create.camera(position=tuple(float(v) for v in cam_pos),
                                          look_at=tuple(float(v) for v in cube_c))
        rgb, dep = grab(str(fc.GetPath()), (640, 480), True)
        save_rgb(rgb, args.out / "cam_head_down_rgb.png")
        save_depth(dep, args.out / "cam_head_down_depth.png",
                   args.out / "cam_head_down_depth_color.png")

        # ---- head LEVEL (tilt 0) via free cam at measured pose ----
        robot.set_head(0.0, 0.0)
        for _ in range(RESETTLE):
            world.step(render=True)
        cp, cf = sensors_mod.camera_world_pose(head_cam_prim)
        look_l = np.array(cp) + 1.5 * np.array(cf)
        print(f"[cam] head(level) pos={np.array(cp).round(3)} fwd={np.array(cf).round(3)}", flush=True)
        fcl = rep.functional.create.camera(position=tuple(float(v) for v in cp),
                                           look_at=tuple(float(v) for v in look_l))
        rgb, _ = grab(str(fcl.GetPath()), (640, 480), False)
        save_rgb(rgb, args.out / "cam_head_level_rgb.png")

        print("[cam] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
