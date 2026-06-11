"""Render R2D3 in a training environment + check that its objects are interactable.

    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene warehouse|kitchen|living_room
    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_scenes.py --scene kitchen --check --no-render

Loads the scene via the R2D3 `setup` hook, holds the robot at the scene's spawn
pose (the free mobile base is re-pinned each step), settles the objects, optionally
verifies they came to rest on their surfaces (`--check`, exits non-zero on failure),
and (unless `--no-render`) writes a screenshot to isaac_sim/tests/captures/.
"""
import argparse
import math
from pathlib import Path

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"


def main() -> int:
    ap = argparse.ArgumentParser(description="Render / check R2D3 in a training scene")
    ap.add_argument("--scene", required=True, help="warehouse | kitchen | living_room")
    ap.add_argument("--check", action="store_true", help="verify objects settle on their surfaces; exit 1 on failure")
    ap.add_argument("--no-render", action="store_true", help="skip the screenshot (fast iteration)")
    ap.add_argument("--dx", type=float, default=3.6)   # camera offset from robot (fallback framing)
    ap.add_argument("--dy", type=float, default=-3.2)
    ap.add_argument("--dz", type=float, default=1.4)
    ap.add_argument("--load-steps", type=int, default=80)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    from isaac_sim.r2d3_sim import scenes
    man = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True, enable_cameras=False,
               setup=lambda w: man.update(scenes.load(args.scene, w)))
    try:
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaac_sim.r2d3_sim import scene as scene_mod
        stage = omni.usd.get_context().get_stage()

        sim.reset()
        sim.go_home()
        if "lift" in man:          # raise arms clear of collidable furniture
            sim.set_lift(float(man["lift"]))
        for _ in range(30):
            sim.world.step(render=False)

        # Hold the robot at the spawn with a FIXED base height (~0.27 m wheel-to-base
        # offset). The free mobile base is re-pinned EVERY step (no gravity/friction to
        # stop residual drift), so `hold()` is used for all post-placement stepping.
        sx, sy, yaw = man.get("spawn", (0.0, 0.0, 0.0))
        q = h.yaw_quat(yaw)
        base_pos = np.array([sx, sy, 0.27])
        # Composed rooms have a visual-only floor -> re-pin the free base each step so
        # arm-reaction doesn't drift it. The warehouse floor is collidable (height != 0)
        # -> let the robot settle onto it instead of forcing z=0.27.
        hold_base = bool(man.get("hold_base", False))

        def hold(n, render=False):
            for _ in range(n):
                if hold_base:
                    sim.set_base_pose(base_pos, q)
                sim.world.step(render=render)

        sim.set_base_pose(base_pos, q)     # one clean teleport to the spawn
        hold(20)
        bp, _ = sim.robot.get_base_pose()
        rmin, rmax = scene_mod.world_range(sim._robot_prim)
        feet_z = float(rmin[2])
        print(f"[scene] robot base={np.asarray(bp).round(2)} feet_z={feet_z:.2f} top_z={rmax[2]:.2f}", flush=True)

        # Floor-snap only the REFERENCED furniture props (varied internal z-offsets);
        # primitive/fixed-box surfaces are already placed correctly.
        for grp in ("/World/k", "/World/lr"):
            gprim = stage.GetPrimAtPath(grp)
            if not gprim.IsValid():
                continue
            for child in gprim.GetChildren():
                if not child.GetChild("ref").IsValid():     # skip primitives/fixed boxes
                    continue
                cp = child.GetPath().pathString
                try:
                    lo, _hi = scene_mod.world_range(cp)
                except Exception:  # noqa: BLE001
                    continue
                if abs(float(lo[2])) > 0.03:
                    UsdGeom.Xformable(child).AddTranslateOp(opSuffix="snap").Set(Gf.Vec3d(0.0, 0.0, -float(lo[2])))
            hold(6)

        hold(60)        # let the manipulable objects fall + settle onto their surfaces

        rc = 0
        if args.check:
            rc = _check(sim, scene_mod, man, feet_z, hold, base_pos)

        if not args.no_render:
            _render(sim, scene_mod, args, man, hold, rmin, rmax)

        return rc
    finally:
        sim.close()


def _check(sim, scene_mod, man, feet_z, hold, base_pos) -> int:
    """Verify each object came to rest, finite, on (near) its support surface, and
    the robot held its spawn. Returns 0 (all PASS) or 1 (any FAIL)."""
    objs = man.get("objects", {})
    surf = man.get("surface_z", {})

    def bottom(prim):
        lo, _ = scene_mod.world_range(prim)
        return float(lo[2])

    z1 = {n: bottom(p) for n, p in objs.items()}
    hold(30)
    z2 = {n: bottom(p) for n, p in objs.items()}

    print("=== interactability check ===", flush=True)
    failed = []
    for n, p in objs.items():
        b = z2[n]
        dz = abs(b - z1[n])
        sz = surf.get(n)
        if sz is None:                                  # rests on the scene floor (near the robot's feet)
            support, on = feet_z, (feet_z - 0.3) <= b <= (feet_z + 0.3)
        else:                                           # rests on a known surface (counter/island/table)
            support, on = sz, (sz - 0.05) <= b <= (sz + 0.10)
        ok = math.isfinite(b) and dz < 0.02 and b > -0.5 and on
        print(f"  [{'PASS' if ok else 'FAIL'}] {n:12s} bottom={b:+.3f} support={support:+.3f} "
              f"drift={dz:.3f} {'' if ok else '<-'}", flush=True)
        if not ok:
            failed.append(n)

    bp = np.asarray(sim.robot.get_base_pose()[0])
    off = float(np.linalg.norm(bp - base_pos)) if np.all(np.isfinite(bp)) else 9e9
    held = off < 0.3                                    # all scenes re-pin the base at the spawn
    print(f"  [{'PASS' if held else 'FAIL'}] robot stable base={bp.round(2)} (off {off:.2f} m)", flush=True)
    if not held:
        failed.append("robot")

    if failed:
        print(f"[check] RESULT: {len(failed)} FAIL -> {failed}", flush=True)
        return 1
    print(f"[check] RESULT: ALL PASS ({len(objs)} objects rest on their surfaces, robot stable)", flush=True)
    return 0


def _render(sim, scene_mod, args, man, hold, rmin, rmax):
    import omni.replicator.core as rep
    from PIL import Image
    ctr = (np.asarray(rmin) + np.asarray(rmax)) / 2.0
    look = np.array(man.get("look", (float(ctr[0]), float(ctr[1]), float(ctr[2]))), dtype=float)
    eye = np.array(man.get("eye", (look[0] + args.dx, look[1] + args.dy, look[2] + args.dz)), dtype=float)
    if "look" in man:      # composed scenes sit on z=0; raise if the robot is elevated
        lift = max(0.0, float(rmin[2]) - 0.1)
        look[2] += lift
        eye[2] += lift
    cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                        look_at=tuple(float(v) for v in look))
    rp = rep.create.render_product(str(cam.GetPath()), (1280, 720))
    ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
    hold(args.load_steps, render=True)        # warm up RTX + stream cloud assets
    a = h.rgba_to_rgb(np.asarray(ann.get_data(do_array_copy=True)))
    out = OUT / f"scene_{args.scene}.png"
    Image.fromarray(a).save(out)
    print(f"[scene] wrote {out.name}  shape={a.shape} mean={float(a.mean()):.0f}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
