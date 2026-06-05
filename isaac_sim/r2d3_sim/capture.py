"""Headless frame capture for the R2D3 bring-up scene.

Renders two views in one pass and writes them as PNGs:

  * a third-person shot framing the WHOLE robot (base/wheels -> raised arms),
    standing on a visual ground plane, with a preview workbench + cube
  * the head-mounted D435 RGB + depth, aimed so the workbench + cube land
    centered in frame (so the head view is actually informative)

Viewport capture does NOT work headless in Isaac Sim 6.0 (the active
viewport is a GUI widget that returns None). The supported path is
omni.replicator: render product + rgb / distance_to_camera annotators +
orchestrator warm-up steps + get_data -> numpy -> PIL.

The workbench + cube are VISUAL-ONLY preview props (no physics) placed on
the head camera's measured optical axis, so they're centered in the head
view regardless of the exact kinematics. The physics-interactive eval
scene (spawned from task YAML) arrives with the engine milestone.

Usage:
    scripts/isaacsim_ros2.sh isaac_sim/r2d3_sim/capture.py [--out DIR]
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# The robot is posed via Robot.go_home() (single source of truth in robot.py).
# NOTE for tuning: lift=0.30 + a shallow-elbow tucked pose rendered BLANK (the
# render-side transforms degenerate while USD stays valid — bisected via
# diag_render.py); the home pose (lift 0.5, elbow joint4=-1.2) renders cleanly.

# Distance along the head camera's optical axis to place the cube.
WORKBENCH_LOOK_DIST_M = 0.85

CUBE_SIZE_M = 0.06
# A normal table height in front of the robot for the head-cam manipulation
# view. The head view is rendered by a free camera aimed DOWN at the cube
# (what the D435 will see once the head-tilt bug — tilt currently rolls the
# camera — is fixed), so the table needs to sit below head height.
TABLE_TOP_Z = 0.55
TABLE_DIST_M = 0.70

SETTLE_STEPS = 150
WARMUP_RENDERS = 16


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(prog="capture", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=Path,
                   default=_REPO_ROOT / "isaac_sim/tests/captures")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("PRIVACY_CONSENT", "Y")

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": True})

    try:
        import numpy as np
        import omni.replicator.core as rep
        from PIL import Image

        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        robot_prim_path = scene_mod.assemble(world)

        # Robot vertical extent (wheels) is pose-independent — read it pre-reset
        # off the USD for the ground height.
        robot_min0, _robot_max0 = scene_mod.world_range(robot_prim_path)
        ground_z = float(robot_min0[2])

        # Head D435 camera prim (used to MEASURE the settled head pose; the
        # head VIEW is rendered through a free camera placed at that pose).
        head_cam_prim = sensors_mod._ensure_camera_prim()

        world.reset()

        robot = Robot(prim_path=robot_prim_path)
        robot.initialize()   # configures position drives + disables gravity

        # ---- Command home; the position drives settle the robot there ------
        robot.go_home()
        print(f"[capture] commanding home, settling {SETTLE_STEPS} steps...", flush=True)
        for _ in range(SETTLE_STEPS):
            world.step(render=True)
        print(f"[capture] settled at home: lift={robot.get_lift_m():.3f} m", flush=True)

        # ---- Place a table + cube in front of the robot (post-settle) ------
        cam_pos, cam_fwd = sensors_mod.camera_world_pose(head_cam_prim)
        # Project the head forward onto the ground plane so the table sits in
        # front of the robot at a sensible spot regardless of camera tilt/roll.
        hfwd = np.array([cam_fwd[0], cam_fwd[1], 0.0])
        hfwd /= np.linalg.norm(hfwd)
        table_xy = cam_pos[:2] + TABLE_DIST_M * hfwd[:2]
        top_z = max(TABLE_TOP_Z, ground_z + 0.30)
        bench_h = top_z - ground_z
        bench_center = (float(table_xy[0]), float(table_xy[1]), ground_z + bench_h / 2.0)
        bench_size = (0.45, 0.45, bench_h)
        cube_center = (float(table_xy[0]), float(table_xy[1]), top_z + CUBE_SIZE_M / 2.0)
        print(f"[capture] head cam at {cam_pos.round(3)} fwd {cam_fwd.round(3)}; "
              f"table_top_z={top_z:.3f} cube {np.array(cube_center).round(3)}", flush=True)

        scene_mod.add_visual_box("/Preview/ground",
                                 (float(table_xy[0]) * 0.5, float(table_xy[1]) * 0.5, ground_z - 0.01),
                                 (6.0, 6.0, 0.02), (0.26, 0.31, 0.40))   # mid blue-grey
        scene_mod.add_visual_box("/Preview/workbench", bench_center, bench_size,
                                 (0.55, 0.40, 0.26))
        scene_mod.add_visual_box("/Preview/cube_red", cube_center,
                                 (CUBE_SIZE_M, CUBE_SIZE_M, CUBE_SIZE_M),
                                 (0.85, 0.10, 0.10))

        # ---- Third-person camera framing robot + props ---------------------
        robot_min, robot_max = scene_mod.world_range(robot_prim_path)
        u_min = np.minimum.reduce([robot_min, np.array(bench_center) - np.array(bench_size) / 2,
                                   np.array(cube_center) - CUBE_SIZE_M / 2])
        u_max = np.maximum.reduce([robot_max, np.array(bench_center) + np.array(bench_size) / 2,
                                   np.array(cube_center) + CUBE_SIZE_M / 2])
        center = (u_min + u_max) / 2.0
        radius = 0.5 * float(np.linalg.norm(u_max - u_min))
        direction = np.array([0.62, -0.72, 0.31])      # front, right, slightly above
        direction /= np.linalg.norm(direction)
        eye = center + max(3.3, 2.3 * radius) * direction
        # Do NOT override focalLength/aperture on a rep camera — it blanks the
        # entire render (rep's internal projection gets corrupted; verified via
        # diag_render.py). Reframe by changing distance, not focal length.
        tp = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                          look_at=tuple(float(v) for v in center))
        tp_cam_prim = str(tp.GetPath())
        print(f"[capture] scene bbox {u_min.round(2)}..{u_max.round(2)}; "
              f"3p eye {eye.round(2)} -> {center.round(2)} cam={tp_cam_prim}", flush=True)

        # ---- Capture helper (low-level, synchronous) -----------------------
        def grab(camera_path, resolution, want_depth):
            rp = rep.create.render_product(camera_path, resolution)
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach(rp)
            depth_annot = None
            if want_depth:
                depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
                depth_annot.attach(rp)
            for _ in range(WARMUP_RENDERS):
                rep.orchestrator.step(pause_timeline=True)
            rgb = rgb_annot.get_data(do_array_copy=True)
            depth = depth_annot.get_data(do_array_copy=True) if depth_annot else None
            rgb_annot.detach()
            if depth_annot:
                depth_annot.detach()
            rp.destroy()
            return rgb, depth

        def save_rgb(rgb, path):
            arr = np.asarray(rgb)
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[:, :, :3]
            arr = arr.astype(np.uint8)
            Image.fromarray(arr, mode="RGB").save(path)
            print(f"[capture] wrote {path}  shape={arr.shape} "
                  f"min={arr.min()} max={arr.max()}", flush=True)

        def save_depth(depth, gray_path, color_path, lo=0.2, hi=1.8):
            d = np.asarray(depth, dtype=np.float32)
            if d.ndim == 3:
                d = d[:, :, 0]
            finite = d[np.isfinite(d)]
            if finite.size:
                print(f"[capture] depth range: {finite.min():.3f}..{finite.max():.3f} m",
                      flush=True)
            clipped = np.clip(np.nan_to_num(d, nan=hi, posinf=hi), lo, hi)
            norm = ((clipped - lo) / (hi - lo) * 255.0).astype(np.uint8)
            Image.fromarray(norm, mode="L").save(gray_path)
            print(f"[capture] wrote {gray_path}", flush=True)
            try:
                import matplotlib.cm as cm
                rgba = (cm.get_cmap("viridis")(norm / 255.0) * 255).astype(np.uint8)
                Image.fromarray(rgba[:, :, :3], mode="RGB").save(color_path)
                print(f"[capture] wrote {color_path}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[capture] depth colorize skipped: {e}", flush=True)

        print("[capture] rendering third-person view...", flush=True)
        tp_rgb, _ = grab(tp_cam_prim, (1280, 720), want_depth=False)
        save_rgb(tp_rgb, args.out / "scene_3p.png")

        # The articulation-child head camera renders from its REST transform
        # in this capture path (physics transforms aren't flushed to a camera
        # prim parented under the articulation), so it looks from the wrong
        # place. Render the head view through a FREE camera placed at the head
        # camera's measured world pose instead. NOTE: M2's live ROS camera uses
        # the mounted camera and will need the real fix (tracking physics on the
        # mounted camera) — this is a capture-only workaround.
        print("[capture] rendering head D435 view (free cam, aimed down at cube)...", flush=True)
        head_view = rep.functional.create.camera(
            position=tuple(float(v) for v in cam_pos),
            look_at=tuple(float(v) for v in np.array(cube_center)),
        )
        head_rgb, head_depth = grab(str(head_view.GetPath()), (640, 480), want_depth=True)
        save_rgb(head_rgb, args.out / "head_d435_rgb.png")
        save_depth(head_depth,
                   args.out / "head_d435_depth.png",
                   args.out / "head_d435_depth_color.png")

        print(f"[capture] DONE — frames in {args.out}", flush=True)
        return 0

    finally:
        sim_app.close()


if __name__ == "__main__":
    sys.exit(main())
