"""Render a third-person GIF walking through each part of the robot moving, so a
human can eyeball that everything actuates.

    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_motion_gif.py [--ee dexterous|gripper]

Phases (fixed base): arms sweep -> head pan/tilt -> body lift up/down ->
hands close/open -> left-arm IK reach. Writes
isaac_sim/tests/captures/motion_<ee>.gif (+ a few stills). The base-drive GIF is
move_task.py (it needs the mobile build).
"""
import argparse
from pathlib import Path

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"


def main() -> int:
    ap = argparse.ArgumentParser(description="Render a full-body motion GIF")
    ap.add_argument("--ee", choices=["dexterous", "gripper"], default="dexterous")
    args = ap.parse_args()
    ee = args.ee
    OUT.mkdir(parents=True, exist_ok=True)

    sim = R2D3(end_effector=ee, headless=True, enable_cameras=False)
    try:
        import omni.replicator.core as rep
        from PIL import Image
        from isaac_sim.r2d3_sim import scene as scene_mod

        sim.reset()
        sim.go_home()
        for _ in range(30):
            sim.world.step(render=False)

        # Frame the whole robot from its world bounding box.
        rmin, rmax = scene_mod.world_range(sim._robot_prim)
        rmin = np.asarray(rmin, float); rmax = np.asarray(rmax, float)
        ctr = (rmin + rmax) / 2.0
        floor_z = float(rmin[2])

        # Coloured studio so the white robot stands out.
        h.set_lighting(dome=130.0, key=3600.0, fill=2500.0)
        scene_mod.add_visual_box("/Preview/floor", (float(ctr[0]), float(ctr[1]), floor_z - 0.01),
                                 (16.0, 16.0, 0.02), (0.22, 0.36, 0.50))
        scene_mod.add_visual_box("/Preview/wall_back", (float(ctr[0]) - 2.6, float(ctr[1]), floor_z + 1.4),
                                 (0.1, 16.0, 5.0), (0.13, 0.16, 0.24))
        scene_mod.add_visual_box("/Preview/wall_side", (float(ctr[0]), float(ctr[1]) + 3.2, floor_z + 1.4),
                                 (16.0, 0.1, 5.0), (0.17, 0.13, 0.22))

        # Front-right elevated 3/4 view (robot faces +X).
        eye = ctr + np.array([3.0, -2.4, 0.6])
        cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                            look_at=tuple(float(v) for v in ctr))
        rp = rep.create.render_product(str(cam.GetPath()), (960, 540))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)

        frames = []

        def grab():
            a = np.asarray(ann.get_data(do_array_copy=True))
            frames.append(Image.fromarray(h.rgba_to_rgb(a)).resize((480, 270)))

        def run(n, every=1):
            for i in range(n):
                sim.world.step(render=True)
                if i % every == 0:
                    grab()

        for _ in range(16):                 # RTX warm-up (no grab)
            sim.world.step(render=True)
        run(4)

        # ---- ARMS sweep (both) ----
        for q in ([0.6, -0.7, 0.6, -1.1, 0.6, 0.7, 0.5],
                  [-0.5, 0.3, -0.5, -0.5, -0.4, -0.3, -0.3],
                  [0.0] * 7):
            sim.set_arm_joints("left", q)
            sim.set_arm_joints("right", q)
            run(26)

        # ---- HEAD pan/tilt ----
        for pan, tilt in [(0.6, -0.2), (-0.6, 0.2), (0.0, -0.4), (0.0, 0.0)]:
            sim.set_head(pan, tilt)
            run(16)

        # ---- LIFT up/down ----
        for height in (0.92, 0.30, 0.55):
            sim.set_lift(height)
            run(28)

        # ---- HANDS close/open ----
        for frac in (1.0, 0.0):
            sim.set_gripper("left", frac)
            sim.set_gripper("right", frac)
            run(22)

        # ---- IK reach (left EE Cartesian) ----
        ee0, _ = sim.get_ee_pose("left")
        sim.set_arm_pose("left", ee0 + np.array([0.10, 0.0, -0.18]), sim.top_down_quat)
        run(26)
        sim.go_home()
        run(18)

        ann.detach(); rp.destroy()
        if frames:
            means = [float(np.asarray(f).mean()) for f in frames]
            gif = OUT / f"motion_{ee}.gif"
            frames[0].save(gif, save_all=True, append_images=frames[1:], duration=60, loop=0)
            frames[0].save(OUT / f"motion_{ee}_first.png")
            frames[len(frames) // 2].save(OUT / f"motion_{ee}_mid.png")
            frames[-1].save(OUT / f"motion_{ee}_last.png")
            print(f"[gif] wrote {gif.name}  frames={len(frames)}  "
                  f"brightness min/mean/max={min(means):.0f}/{np.mean(means):.0f}/{max(means):.0f}", flush=True)
        print("[gif] DONE", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
