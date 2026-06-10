"""Validate the dexterous hand: robot init (41 DOF), IK top-down, open/close the
hand, render it, and measure fingertip positions relative to l_hand_link (for the
grasp point)."""
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
OUT = _REPO / "isaac_sim/tests/captures"


def mat_to_quat(R):
    import numpy as np
    t = np.trace(R)
    if t > 0:
        s = 0.5/np.sqrt(t+1.0); return np.array([0.25/s,(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s=2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]); return np.array([(R[2,1]-R[1,2])/s,0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
    if i == 1:
        s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]); return np.array([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s])
    s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]); return np.array([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s])


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import UsdGeom, Gf
        from isaacsim.core.api import World
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        print(f"[dex] robot init OK: {robot.num_dof} DOFs", flush=True)
        stage = omni.usd.get_context().get_stage()
        f = lambda n: next((p.GetPath().pathString for p in stage.Traverse()
                            if p.GetName()==n and p.GetTypeName()=="Xform"), None)
        def wp(path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            o = m.Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])

        HAND = f("l_hand_link")
        tips = {n: f("l_dex_"+n) for n in ["thumb_tip","index_tip","middle_tip","ring_tip","pinky_tip"]}

        # IK left arm to a top-down pose
        bo = wp(f("base_link_underpan"))
        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        ik.set_robot_base_pose(bo, np.array([1.,0.,0.,0.]))
        quat = mat_to_quat(np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
        q,ok = ik.compute_inverse_kinematics("l_hand_link", np.array([0.5,-0.22,0.55]), quat,
                                             position_tolerance=0.01, orientation_tolerance=0.1)
        print(f"[dex] IK ok={ok}", flush=True)
        robot.go_home()
        for _ in range(120):
            world.step(render=False)
        for _ in range(160):
            robot.set_arm_targets("left", list(q)); robot.set_finger("left", 0.035)  # open
            world.step(render=False)
        h = wp(HAND)
        print(f"[dex] OPEN hand={h.round(3)}", flush=True)
        for n,p in tips.items():
            t = wp(p); print(f"[dex]   tip {n}: {t.round(3)}  off-from-hand={ (t-h).round(3) }", flush=True)

        def shot(name):
            d = np.array([0.5,-0.7,0.3]); d/=np.linalg.norm(d)
            cam = rep.functional.create.camera(position=tuple(float(v) for v in (h+0.4*d)), look_at=tuple(float(v) for v in h))
            rp = rep.create.render_product(str(cam.GetPath()), (900,600))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(14): world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:,:,:3].astype(np.uint8),"RGB").save(OUT/name)
            a.detach(); rp.destroy(); print(f"[dex] wrote {name}", flush=True)
        shot("dex_open.png")

        # close the hand
        for _ in range(120):
            robot.set_arm_targets("left", list(q)); robot.set_finger("left", 0.0)  # closed
            world.step(render=False)
        print("[dex] CLOSED:", flush=True)
        for n,p in tips.items():
            t = wp(p); print(f"[dex]   tip {n}: {t.round(3)}", flush=True)
        shot("dex_closed.png")
        print("[dex] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
