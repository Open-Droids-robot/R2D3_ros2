"""Probe the head D435 geometry: does head_joint2 tilt PITCH or ROLL the camera?

Render-free. Settles the robot, then for several (lift, pan, tilt) head
configs measures the head camera prim's world pose and prints:
  - position (does it move with lift/pan? -> camera tracks the head in USD)
  - forward = world image of camera local -Z (view direction)
  - up      = world image of camera local +Y
If tilt changes forward.z -> it PITCHES (good). If forward.z stays ~0 and
only `up` rotates -> it ROLLS (the bug).
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
try:
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.core.api import World
    from isaac_sim.r2d3_sim import scene as scene_mod
    from isaac_sim.r2d3_sim import sensors as sensors_mod
    from isaac_sim.r2d3_sim.robot import Robot

    world = World(stage_units_in_meters=1.0)
    rpath = scene_mod.assemble(world)
    head_cam = sensors_mod._ensure_camera_prim()
    world.reset()
    robot = Robot(prim_path=rpath); robot.initialize(); robot._art.disable_gravity()
    robot.lock_agv_wheels()
    stage = omni.usd.get_context().get_stage()

    def settle(lift, pan, tilt, n=30):
        for _ in range(n):
            robot.hold_agv_wheels()
            robot.set_lift_m(lift)
            robot.set_head(pan, tilt)
            robot.set_arm_targets("left", [0.0]*7)
            robot.set_arm_targets("right", [0.0]*7)
            world.step(render=False)
            robot.hold_agv_wheels()
            robot._art.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float32))

    def probe(label, lift, pan, tilt):
        settle(lift, pan, tilt)
        cache = UsdGeom.XformCache()
        m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(head_cam))
        p0 = m.Transform(Gf.Vec3d(0, 0, 0))
        fwd = np.array(m.Transform(Gf.Vec3d(0, 0, -1))) - np.array(p0)
        up = np.array(m.Transform(Gf.Vec3d(0, 1, 0))) - np.array(p0)
        fwd /= np.linalg.norm(fwd); up /= np.linalg.norm(up)
        print(f"[head] {label:28s} pos=({p0[0]:.3f},{p0[1]:.3f},{p0[2]:.3f}) "
              f"fwd=({fwd[0]:.3f},{fwd[1]:.3f},{fwd[2]:.3f}) "
              f"up=({up[0]:.3f},{up[1]:.3f},{up[2]:.3f})", flush=True)

    print("[head] === head_joint2 tilt sweep (lift 0.5, pan 0) ===", flush=True)
    for tilt in (-0.4, -0.2, 0.0, 0.2, 0.4):
        probe(f"tilt={tilt:+.2f}", 0.5, 0.0, tilt)
    print("[head] === pan sweep (lift 0.5, tilt 0) ===", flush=True)
    for pan in (-0.5, 0.0, 0.5):
        probe(f"pan={pan:+.2f}", 0.5, pan, 0.0)
    print("[head] === lift sweep (pan/tilt 0) ===", flush=True)
    for lift in (0.2, 0.5, 0.8):
        probe(f"lift={lift:.2f}", lift, 0.0, 0.0)
    print("[head] DONE", flush=True)
finally:
    app.close()
