"""Isolate the NaN source. Three tests in one boot:
  R1: gravity OFF, no drive, zero velocity, just step  -> if NaN, the
      articulation itself is numerically broken (bad inertia/mass matrix).
  R2: gravity OFF, position drive (g300), command home -> drive-only.
  R3: gravity ON,  position drive (g300), command home -> full.
Self-collisions OFF + solver iters authored pre-reset.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

ROOT_PRIM = "/r2d3_v1/Geometry/base_link_underpan"
GAINS = dict(arm=(300, 50), head=(80, 15), lift=(8000, 800), finger=(40, 5))


def group_of(name):
    if name.startswith(("l_joint", "r_joint")): return "arm"
    if name.startswith("head_joint"): return "head"
    if name == "platform_joint": return "lift"
    return "finger"


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from pxr import PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.utils.types import ArticulationAction
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot
        from isaac_sim.r2d3_sim import robot as robot_mod

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        stage = omni.usd.get_context().get_stage()
        papi = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(ROOT_PRIM))
        papi.CreateEnabledSelfCollisionsAttr(False)
        papi.CreateSolverPositionIterationCountAttr(32)
        papi.CreateSolverVelocityIterationCountAttr(4)

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        art = robot._art
        dof = art.dof_names; n = art.num_dof
        idx = robot.joint_index
        ctrl = art.get_articulation_controller()
        zeros = np.zeros(n, dtype=np.float32)

        tgt = np.zeros(n, dtype=np.float32)
        tgt[idx[robot_mod.t.LIFT_JOINT]] = robot_mod.HOME_LIFT_M
        for j, nm in enumerate(robot_mod.t.LEFT_ARM_JOINTS): tgt[idx[nm]] = robot_mod.HOME_LEFT_ARM[j]
        for j, nm in enumerate(robot_mod.t.RIGHT_ARM_JOINTS): tgt[idx[nm]] = robot_mod.HOME_RIGHT_ARM[j]
        kps = np.array([GAINS[group_of(nm)][0] for nm in dof], dtype=np.float32)
        kds = np.array([GAINS[group_of(nm)][1] for nm in dof], dtype=np.float32)

        def run(label, gravity, drive, steps=200):
            art.set_joint_positions(tgt); art.set_joint_velocities(zeros)
            (art.enable_gravity if gravity else art.disable_gravity)()
            if drive:
                ctrl.switch_control_mode("position"); ctrl.set_gains(kps=kps, kds=kds)
            else:
                ctrl.set_gains(kps=zeros, kds=zeros)   # free joints
            first_nan = -1
            for s in range(steps):
                if drive:
                    ctrl.apply_action(ArticulationAction(joint_positions=tgt))
                world.step(render=False)
                if first_nan < 0:
                    q = art.get_joint_positions()
                    if int(np.sum(~np.isfinite(q))) > 0:
                        first_nan = s
            q = art.get_joint_positions()
            nan = int(np.sum(~np.isfinite(q)))
            qq = np.nan_to_num(q)
            print(f"[drv] {label:18s} nan={nan:2d} first_nan_step={first_nan:4d} "
                  f"|q|max={np.abs(qq).max():.3f} lift={qq[idx['platform_joint']]:.3f}",
                  flush=True)

        run("R1_norest_nograv", gravity=False, drive=False)
        run("R2_drive_nograv",  gravity=False, drive=True)
        run("R3_drive_grav",    gravity=True,  drive=True)
        print("[drv] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
