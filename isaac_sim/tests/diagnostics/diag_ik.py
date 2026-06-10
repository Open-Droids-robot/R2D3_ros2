"""Validate Lula IK for R2D3's left arm.

(1) Solve IK for the known f_low gripper pose -> should reproduce it.
(2) Solve IK for a TOP-DOWN grasp pose over a cube spot in front -> apply in sim
    and confirm l_hand_link reaches the target.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

URDF = str(_REPO / "isaac_sim/urdf/r2d3_v1.urdf")
DESC = str(_REPO / "isaac_sim/urdf/r2d3_left_arm_lula.yaml")
EE = "l_hand_link"


def mat_to_quat(R):
    import numpy as np
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0); w = 0.25/s
        x = (R[2,1]-R[1,2])*s; y = (R[0,2]-R[2,0])*s; z = (R[1,0]-R[0,1])*s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = 2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]); w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
        elif i == 1:
            s = 2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]); w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
        else:
            s = 2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]); w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    q = np.array([w, x, y, z]); return q/np.linalg.norm(q)


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaacsim.core.api import World
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        stage = omni.usd.get_context().get_stage()
        hand_path = next((p.GetPath().pathString for p in stage.Traverse()
                          if p.GetName() == "l_hand_link" and p.GetTypeName() == "Xform"), None)
        base_path = next((p.GetPath().pathString for p in stage.Traverse()
                          if p.GetName() == "base_link_underpan" and p.GetTypeName() == "Xform"), None)

        def hand_world():
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(hand_path))
            o = np.array(m.Transform(Gf.Vec3d(0, 0, 0)))
            X = np.array(m.Transform(Gf.Vec3d(1, 0, 0))) - o
            return o, X/np.linalg.norm(X)

        # base pose for Lula
        bm = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(base_path))
        bo = np.array(bm.Transform(Gf.Vec3d(0, 0, 0)))
        print(f"[ik] base_link_underpan world pos={bo.round(3)}", flush=True)

        # set lift to 0.5 (matches the yaml) and settle
        robot.go_home()
        for _ in range(200):
            robot.set_arm_targets("left", [0.0, -1.3, 0.0, -0.6, 0.0, 0.9, 0.0])
            world.step(render=False)
        known_pos, known_x = hand_world()
        print(f"[ik] f_low actual l_hand_link pos={known_pos.round(3)} approachX={known_x.round(2)}", flush=True)

        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        print(f"[ik] solver frames include l_hand_link: {'l_hand_link' in ik.get_all_frame_names()}", flush=True)
        ik.set_robot_base_pose(bo, np.array([1.0, 0.0, 0.0, 0.0]))

        # (1) reach the known f_low position (no orientation constraint first)
        q, ok = ik.compute_inverse_kinematics(EE, known_pos, None, position_tolerance=0.005)
        print(f"[ik] (1) reach known pos: success={ok} q={np.round(q,3) if q is not None else None}", flush=True)
        if ok:
            fk_pos, _ = ik.compute_forward_kinematics(EE, q)
            print(f"[ik]     FK of solution={np.round(fk_pos,3)} (target {known_pos.round(3)})", flush=True)

        # (2) TOP-DOWN grasp orientation: link +X -> world -Z, link +Y -> world +Y
        R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)  # cols = X,Y,Z axes in world
        quat = mat_to_quat(R)
        for tgt in ([0.45, -0.20, 0.55], [0.45, -0.20, 0.45], [0.50, -0.22, 0.50]):
            q, ok = ik.compute_inverse_kinematics(EE, np.array(tgt), quat,
                                                  position_tolerance=0.01, orientation_tolerance=0.2)
            print(f"[ik] (2) top-down @ {tgt}: success={ok}", flush=True)
            if ok:
                # apply in sim and check
                for _ in range(200):
                    robot.set_arm_targets("left", list(q)); world.step(render=False)
                ap, ax = hand_world()
                print(f"[ik]     applied -> l_hand_link={ap.round(3)} approachX={ax.round(2)} "
                      f"(target {tgt}, want approachX~[0,0,-1])", flush=True)
        print("[ik] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
