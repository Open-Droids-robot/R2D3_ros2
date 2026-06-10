"""Render a contact sheet of candidate home poses to pick a clean one.

The right arm base is mounted 180 deg yawed vs the left (r_joint1 rpy has a
+pi), so identical L/R joint values splay the arms. This tries several
candidates (including mirrored-right variants) at head-neutral, lift 0.5,
front 3/4 camera, and tiles them into one labeled PNG.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
OUT = _REPO / "isaac_sim/tests/captures"; OUT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

# (label, left_arm[7], right_arm[7], head_pan, head_tilt)
Z = [0.0] * 7
POSES = [
    ("zeros",        Z, Z, 0.0, 0.0),
    ("fwd_sym",      [0,-0.3,0,-0.9,0,0.6,0], [0,-0.3,0,-0.9,0,0.6,0], 0.0, 0.0),
    ("fwd_mirror",   [0,-0.3,0,-0.9,0,0.6,0], [0, 0.3,0, 0.9,0,-0.6,0], 0.0, 0.0),
    ("home_current", [0,-0.5,0,-1.2,0,0.7,0], [0,-0.5,0,-1.2,0,0.7,0], 0.0, 0.0),
    ("down_gentle",  [0,-0.8,0,-0.5,0,0.3,0], [0,-0.8,0,-0.5,0,0.3,0], 0.0, 0.0),
    ("tuck_in",      [0.3,-0.3,0,-1.0,0,0.6,0], [-0.3,-0.3,0,-1.0,0,0.6,0], 0.0, 0.0),
]


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        from PIL import Image, ImageDraw
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize(); robot._art.disable_gravity()
        robot.lock_agv_wheels()

        def settle(L, R, pan, tilt, n=60):
            for _ in range(n):
                robot.hold_agv_wheels()
                robot.set_lift_m(0.5)
                robot.set_head(pan, tilt)
                robot.set_arm_targets("left", L)
                robot.set_arm_targets("right", R)
                world.step(render=True)
                robot.hold_agv_wheels()
                robot._art.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float32))

        def grab():
            cam = rep.functional.create.camera(position=(2.8, -1.0, 0.9),
                                               look_at=(0.0, 0.0, 0.7))
            rp = rep.create.render_product(str(cam.GetPath()), (480, 360))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(14):
                rep.orchestrator.step(pause_timeline=True)
            d = np.asarray(a.get_data(do_array_copy=True))
            a.detach(); rp.destroy()
            return d[:, :, :3].astype(np.uint8) if d.ndim == 3 and d.shape[2] == 4 else d.astype(np.uint8)

        tiles = []
        for (label, L, R, pan, tilt) in POSES:
            settle(L, R, pan, tilt)
            img = grab()
            print(f"[poses] {label}: min={img.min()} max={img.max()}", flush=True)
            pim = Image.fromarray(img, mode="RGB")
            ImageDraw.Draw(pim).text((6, 6), label, fill=(0, 0, 0))
            tiles.append(pim)

        cols, rows = 3, 2
        w, h = tiles[0].size
        sheet = Image.new("RGB", (cols * w, rows * h), (255, 255, 255))
        for i, pim in enumerate(tiles):
            sheet.paste(pim, ((i % cols) * w, (i // cols) * h))
        sheet.save(OUT / "pose_grid.png")
        print(f"[poses] wrote {OUT/'pose_grid.png'}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
