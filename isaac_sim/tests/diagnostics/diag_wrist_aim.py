"""Measure the exact wrist-camera aim toward the grasp, in l_link7's frame, so
we can bake the right mount rpy. IKs the left hand over a cube, then reports the
direction from the wrist camera to the grasp point expressed in l_link7-local,
and the rpy that aims camera_link +X (the D435 forward) along it."""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")
OUT = _REPO / "isaac_sim/tests/captures"
URDF = str(_REPO / "isaac_sim/urdf/r2d3_v1.urdf")
DESC = str(_REPO / "isaac_sim/urdf/r2d3_left_arm_lula.yaml")


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
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        gmin, _ = scene_mod.world_range(rpath); gz = float(gmin[2])
        TT = 0.44; CUBE = 0.04; CC = (0.55, -0.22, TT + CUBE/2)
        FixedCuboid(prim_path="/World/table", name="table", position=np.array([0.6, -0.2, gz+(TT-gz)/2]),
                    scale=np.array([0.4, 0.4, TT-gz]), color=np.array([0.62, 0.46, 0.30]))
        DynamicCuboid(prim_path="/World/cube", name="cube", position=np.array(CC),
                      scale=np.array([CUBE]*3), color=np.array([0.9, 0.12, 0.12]), mass=0.03)
        scene_mod.add_visual_box("/Preview/ground", (0.3, -0.1, gz-0.012), (8, 8, 0.02), (0.30, 0.34, 0.42))

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        scene_mod._hide_legacy_hand_flanges()
        stage = omni.usd.get_context().get_stage()
        find = lambda n: next((p for p in stage.Traverse() if p.GetName()==n), None)
        def xf(name): return UsdGeom.XformCache().GetLocalToWorldTransform(find(name))
        def wp(name):
            o = xf(name).Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])

        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        ik.set_robot_base_pose(wp("base_link_underpan"), np.array([1.,0.,0.,0.]))
        quat = mat_to_quat(np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
        q,ok = ik.compute_inverse_kinematics("l_hand_link", np.array([CC[0]-0.12, CC[1], CC[2]+0.10]), quat,
                                             position_tolerance=0.02, orientation_tolerance=0.2)
        robot.go_home()
        for _ in range(120): world.step(render=False)
        if ok:
            for _ in range(220):
                robot.set_arm_targets("left", list(q)); robot.set_finger("left", 0.0); world.step(render=False)
        print(f"[aim] IK ok={ok}", flush=True)

        cam_pos = wp("l_wrist_camera_link")
        # target the OBJECT in the workspace (the approach/grasp region in front of
        # the hand) — the held point between the fingers is occluded by the hand.
        grasp = np.array(CC)
        # direction in l_link7 local frame
        L7 = np.array(xf("l_link7"))  # 4x4 row-vectors (Gf), columns are axes
        # build rotation (world axes of l_link7)
        o7 = wp("l_link7")
        ax = lambda v: np.array(xf("l_link7").Transform(Gf.Vec3d(*v))) - o7
        Rx, Ry, Rz = ax((1,0,0)), ax((0,1,0)), ax((0,0,1))
        Rmat = np.column_stack([Rx, Ry, Rz])           # world<-local
        d_world = grasp - cam_pos; d_world /= np.linalg.norm(d_world)
        d_local = Rmat.T @ d_world                      # local<-world
        d_local /= np.linalg.norm(d_local)
        pitch = -np.arcsin(np.clip(d_local[2], -1, 1))
        yaw = np.arctan2(d_local[1], d_local[0])
        print(f"[aim] cam={cam_pos.round(3)} grasp(mid_tip)={grasp.round(3)}", flush=True)
        print(f"[aim] dir_local={d_local.round(3)}", flush=True)
        print(f"[aim] >>> mount rpy = \"0 {pitch:.4f} {yaw:.4f}\"  (roll 0, pitch {np.degrees(pitch):.1f}deg, yaw {np.degrees(yaw):.1f}deg)", flush=True)

        # render an AIMED preview (Camera looking straight at the grasp) to confirm framing
        campath = find("l_wrist_camera_link").GetPath().pathString + "/AimCam"
        cam = UsdGeom.Camera.Define(stage, campath)
        cam.GetFocalLengthAttr().Set(20.0); cam.GetHorizontalApertureAttr().Set(36.0)
        cam.GetVerticalApertureAttr().Set(27.0); cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
        # orient so cam -Z looks at grasp: build basis in local frame
        f = d_local; up0 = np.array([0,0,1.0])
        r = np.cross(f, up0); r = r/ (np.linalg.norm(r)+1e-9); u = np.cross(r, f)
        # USD cam looks down -Z; cam +Z = -f
        Rc = np.column_stack([r, u, -f])
        qc = mat_to_quat(Rc)
        x = UsdGeom.Xformable(cam); x.ClearXformOpOrder()
        x.AddOrientOp().Set(Gf.Quatf(float(qc[0]), Gf.Vec3f(float(qc[1]),float(qc[2]),float(qc[3]))))
        rp = rep.create.render_product(campath, (640,480))
        a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
        for _ in range(20): world.step(render=True)
        Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:,:,:3].astype(np.uint8),"RGB").save(OUT/"wrist_aimed.png")
        a.detach(); rp.destroy(); print("[aim] wrote wrist_aimed.png", flush=True)
        print("[aim] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
