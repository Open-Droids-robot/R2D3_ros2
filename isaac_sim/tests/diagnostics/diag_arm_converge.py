"""Does the left arm converge to its commanded joint targets? (and how fast)

Commands f_low in one step and prints actual l_joint positions vs targets + the
hand world x every 50 steps, to see whether the position drive reaches the
targets and whether the hand pose is repeatable/settled.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

POSE = [0.0, -1.3, 0.0, -0.6, 0.0, 0.9, 0.0]


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        stage = omni.usd.get_context().get_stage()
        hand = next((p.GetPath().pathString for p in stage.Traverse()
                     if p.GetName() == "l_hand_link" and p.GetTypeName() == "Xform"), None)

        def hx():
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(hand))
            o = m.Transform(Gf.Vec3d(0, 0, 0)); return np.array([o[0], o[1], o[2]])

        robot.go_home()
        for _ in range(60):
            world.step(render=False)
        names = [f"l_joint{i}" for i in range(1, 8)]
        for s in range(1, 601):
            robot.set_arm_targets("left", POSE)
            world.step(render=False)
            if s % 50 == 0 or s == 1:
                q = robot.get_joint_positions()
                act = np.array([q[n] for n in names])
                err = act - np.array(POSE)
                h = hx()
                print(f"[cv] step {s:3d}: hand=({h[0]:.3f},{h[1]:.3f},{h[2]:.3f}) "
                      f"maxerr={np.abs(err).max():.4f} j2={act[1]:.3f}(t-1.3) j4={act[3]:.3f}(t-0.6)",
                      flush=True)
        print("[cv] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
