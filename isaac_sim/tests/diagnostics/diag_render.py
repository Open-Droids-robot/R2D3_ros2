"""Diagnostic: why does the robot stop rendering? Bisect pose/lift vs props.

Boots once, renders the SAME fixed camera (no props) at two robot poses:
  A: lift 0.5 + diag arm pose      (matches the earlier WORKING diag_A)
  P: lift 0.3 + capture arm pose   (matches the failing capture)
If A shows the robot and P does not, the lift/pose is the cause.
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

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
try:
    import numpy as np
    import omni.replicator.core as rep
    from PIL import Image
    from isaacsim.core.api import World
    from isaac_sim.r2d3_sim import scene as scene_mod
    from isaac_sim.r2d3_sim.robot import Robot

    world = World(stage_units_in_meters=1.0)
    rpath = scene_mod.assemble(world)
    world.reset()
    robot = Robot(prim_path=rpath); robot.initialize(); robot._art.disable_gravity()

    def settle(lift, larm, rarm, n=120):
        for step in range(n):
            robot.hold_agv_wheels()
            if step == 0:
                robot.lock_agv_wheels()
            robot.set_lift_m(lift)
            robot.set_head(0.0, -0.3)
            robot.set_arm_targets("left", larm)
            robot.set_arm_targets("right", rarm)
            world.step(render=True)
            robot.hold_agv_wheels()
            robot._art.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float32))

    def grab_save(name):
        cam = rep.functional.create.camera(position=(1.8, -1.8, 1.6), look_at=(0.0, 0.0, 0.9))
        rp = rep.create.render_product(str(cam.GetPath()), (960, 720))
        a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
        for _ in range(16):
            rep.orchestrator.step(pause_timeline=True)
        d = np.asarray(a.get_data(do_array_copy=True))
        a.detach(); rp.destroy()
        arr = d[:, :, :3].astype(np.uint8) if d.ndim == 3 and d.shape[2] == 4 else d.astype(np.uint8)
        Image.fromarray(arr, mode="RGB").save(OUT / name)
        print(f"[diag] {name}: min={arr.min()} max={arr.max()} mean={arr.mean():.1f}", flush=True)

    settle(0.5, [0.40, -0.6, 0.0, -1.2, 0.0, 0.6, 0.0], [-0.40, -0.6, 0.0, -1.2, 0.0, 0.6, 0.0])
    print(f"[diag] A lift={robot.get_lift_m():.3f}", flush=True)
    grab_save("diag_A_lift05.png")

    settle(0.3, [0.30, -0.5, 0.0, -1.0, 0.0, 0.8, 0.0], [-0.30, -0.5, 0.0, -1.0, 0.0, 0.8, 0.0])
    print(f"[diag] P lift={robot.get_lift_m():.3f}", flush=True)
    grab_save("diag_P_lift03.png")

    print("[diag] DONE", flush=True)
finally:
    app.close()
