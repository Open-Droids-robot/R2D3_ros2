"""Render R2D3 standing in a training environment (screenshot).

    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene warehouse|kitchen|living_room

Loads the scene via the R2D3 `setup` hook, drops the robot onto the floor at the
scene's spawn pose, frames it third-person, and writes
isaac_sim/tests/captures/scene_<name>.png.
"""
import argparse
from pathlib import Path

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"


def main() -> int:
    ap = argparse.ArgumentParser(description="Render R2D3 in a training scene")
    ap.add_argument("--scene", required=True, help="warehouse | kitchen | living_room")
    ap.add_argument("--dx", type=float, default=3.6)   # camera offset from robot (tunable)
    ap.add_argument("--dy", type=float, default=-3.2)
    ap.add_argument("--dz", type=float, default=1.4)
    ap.add_argument("--load-steps", type=int, default=80)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    from isaac_sim.r2d3_sim import scenes
    spawn = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True, enable_cameras=False,
               setup=lambda w: spawn.update(scenes.load(args.scene, w)))
    try:
        import omni.replicator.core as rep
        from PIL import Image
        from isaac_sim.r2d3_sim import scene as scene_mod

        sim.reset()
        sim.go_home()
        for _ in range(30):
            sim.world.step(render=False)

        # Place the robot at the spawn with a FIXED base height (the wheel-to-base
        # offset is constant for this robot, ~0.25 m). A fixed value beats deriving
        # from the post-reset pose, which a floor collider (e.g. the warehouse) pushes
        # up during reset. The kinematic base then holds this pose. One clean teleport
        # only — a second after wheel/floor contact can spike the solver to NaN.
        sx, sy, yaw = spawn.get("spawn", (0.0, 0.0, 0.0))
        q = h.yaw_quat(yaw)
        sim.set_base_pose(np.array([sx, sy, 0.27]), q)
        for _ in range(20):
            sim.world.step(render=False)

        bp, _ = sim.robot.get_base_pose()
        rmin, rmax = scene_mod.world_range(sim._robot_prim)
        print(f"[scene] robot base={np.asarray(bp).round(2)} feet_z={rmin[2]:.2f} top_z={rmax[2]:.2f}", flush=True)
        import omni.usd
        from pxr import UsdGeom, Gf
        _stg = omni.usd.get_context().get_stage()
        for grp in ("/World/k", "/World/lr"):     # composed furniture groups
            gprim = _stg.GetPrimAtPath(grp)
            if not gprim.IsValid():
                continue
            # snap EACH asset's base onto the floor (assets carry varied z-offsets)
            for child in gprim.GetChildren():
                cp = child.GetPath().pathString
                try:
                    lo, _hi = scene_mod.world_range(cp)
                except Exception:  # noqa: BLE001
                    continue
                if abs(float(lo[2])) > 0.03:
                    UsdGeom.Xformable(child).AddTranslateOp(opSuffix="snap").Set(Gf.Vec3d(0.0, 0.0, -float(lo[2])))
            for _ in range(6):
                sim.world.step(render=False)
            lo, hi = scene_mod.world_range(grp)
            print(f"[scene] {grp} bbox x[{lo[0]:.1f},{hi[0]:.1f}] y[{lo[1]:.1f},{hi[1]:.1f}] z[{lo[2]:.1f},{hi[2]:.1f}]", flush=True)

        # Camera: scene hints if present, else frame the robot's MEASURED centre
        # closely (robust to elevated floors / occlusion — robot prominent, scene
        # as backdrop).
        ctr = (np.asarray(rmin) + np.asarray(rmax)) / 2.0
        look = np.array(spawn.get("look", (float(ctr[0]), float(ctr[1]), float(ctr[2]))), dtype=float)
        eye = np.array(spawn.get("eye", (look[0] + args.dx, look[1] + args.dy, look[2] + args.dz)), dtype=float)
        if "look" in spawn:     # composed scenes sit on z=0; raise if the robot is elevated
            lift = max(0.0, float(rmin[2]) - 0.1)
            look[2] += lift
            eye[2] += lift
        cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                            look_at=tuple(float(v) for v in look))
        rp = rep.create.render_product(str(cam.GetPath()), (1280, 720))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)

        for _ in range(args.load_steps):     # warm up RTX + stream assets from the cloud
            sim.world.step(render=True)

        a = h.rgba_to_rgb(np.asarray(ann.get_data(do_array_copy=True)))
        out = OUT / f"scene_{args.scene}.png"
        Image.fromarray(a).save(out)
        print(f"[scene] wrote {out.name}  shape={a.shape} mean={float(a.mean()):.0f}  "
              f"spawn=({sx},{sy},{yaw})", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
