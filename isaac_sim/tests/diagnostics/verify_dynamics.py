"""Verify the drive refactor: stable hold, target tracking, live force-torque.

No teleport, no velocity-zero. Robot._configure_drives sets position drives +
disables gravity. Checks:
  1. hold home 250 steps -> no NaN, lift holds ~0.5
  2. command l_joint1 -> 0.6 -> tracks; report wrist FT during the move
     (nonzero transient reaction = FT is live, vs the old always-zero)
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        robot.go_home()

        for s in (1, 60, 120, 250):
            target = s - (0 if s == 1 else (60 if s == 60 else (60 if s == 120 else 130)))
            # simpler: step in chunks
        # step to 250 in chunks, report
        cum = 0
        for chunk in (1, 59, 60, 130):
            for _ in range(chunk):
                world.step(render=False)
            cum += chunk
            q = robot.get_joint_positions()
            nan = sum(1 for v in q.values() if v != v)
            print(f"[dyn] hold@{cum:3d}  nan={nan}  lift={q['platform_joint']:.3f}  "
                  f"l_joint1={q['l_joint1']:.4f}  l_joint4={q['l_joint4']:.4f}", flush=True)

        print("[dyn] --- command l_joint1 -> 0.6 (real motion) ---", flush=True)
        robot.set_arm_targets("left", [0.6, -0.0, 0.0, -0.0, 0.0, 0.0, 0.0])
        ftmax = 0.0
        for i in range(150):
            world.step(render=False)
            if i in (5, 20, 60):
                w = robot.get_wrist_wrench("left")
                fmag = float(np.linalg.norm(w[:3])); tmag = float(np.linalg.norm(w[3:]))
                ftmax = max(ftmax, fmag)
                q = robot.get_joint_positions()
                print(f"[dyn]   step {i:3d}: l_joint1={q['l_joint1']:.4f}  "
                      f"wristL |F|={fmag:.3f} N |T|={tmag:.4f} N·m", flush=True)
        q = robot.get_joint_positions()
        print(f"[dyn] settled: l_joint1={q['l_joint1']:.4f} (target 0.6)  "
              f"peak|F|during move={ftmax:.3f} N", flush=True)
        print("[dyn] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
