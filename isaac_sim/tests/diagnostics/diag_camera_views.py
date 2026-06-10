"""Render what each camera SEES: head (camera_link/head_camera) + left wrist.
Sets up a table+cube, IKs the left arm over the cube, and renders the head view,
the left-wrist view, and a third-person — to confirm the cameras are positioned
right (head -> workspace, wrist -> the grasp)."""
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
        findx = lambda n: next((p.GetPath().pathString for p in stage.Traverse() if p.GetName()==n and p.GetTypeName()=="Xform"), None)
        def wp(name):
            o = UsdGeom.XformCache().GetLocalToWorldTransform(find(name)).Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])

        # Camera prim on a given camera link (looks down link +X = the D435 forward)
        def make_cam(link_name, fwd=0.0):
            link = find(link_name)
            path = link.GetPath().pathString + "/Camera"
            if stage.GetPrimAtPath(path): return path
            cam = UsdGeom.Camera.Define(stage, path)
            cam.GetFocalLengthAttr().Set(26.0); cam.GetHorizontalApertureAttr().Set(36.0)
            cam.GetVerticalApertureAttr().Set(27.0); cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 100.0))
            xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(fwd, 0.0, 0.0))   # push lens forward to clear the head shroud
            xf.AddOrientOp().Set(Gf.Quatf(0.5, Gf.Vec3f(0.5, -0.5, -0.5)))
            return path
        head_cam = make_cam("head_camera_link", fwd=0.10)   # camera_link is recessed ~0.08 m inside the head
        lwrist_cam = make_cam("l_wrist_camera_link")

        # IK the left arm so the hand is just over the cube (wrist cam sees the grasp)
        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        ik.set_robot_base_pose(wp("base_link_underpan"), np.array([1.,0.,0.,0.]))
        quat = mat_to_quat(np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
        q,ok = ik.compute_inverse_kinematics("l_hand_link", np.array([CC[0]-0.12, CC[1], CC[2]+0.14]), quat,
                                             position_tolerance=0.02, orientation_tolerance=0.2)
        robot.go_home()
        for _ in range(120): world.step(render=False)
        if ok:
            for _ in range(220):
                robot.set_arm_targets("left", list(q)); robot.set_finger("left", 0.035)
                robot.set_head(0.0, -0.75)   # tilt head down to look at the workspace
                world.step(render=False)
        print(f"[view] IK ok={ok}", flush=True)

        def shoot_cam(path, name):
            rp = rep.create.render_product(path, (640, 480))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(20): world.step(render=True)
            Image.fromarray(np.asarray(a.get_data(do_array_copy=True))[:,:,:3].astype(np.uint8),"RGB").save(OUT/name)
            a.detach(); rp.destroy(); print(f"[view] wrote {name}", flush=True)

        shoot_cam(head_cam, "view_head.png")
        shoot_cam(lwrist_cam, "view_lwrist.png")
        # third-person for context
        ctr = np.array([0.45,-0.2,0.6]); d=np.array([0.7,-0.55,0.35]); d/=np.linalg.norm(d)
        tp = rep.functional.create.camera(position=tuple(float(v) for v in ctr+1.8*d), look_at=tuple(float(v) for v in ctr))
        shoot_cam(str(tp.GetPath()), "view_third.png")
        print("[view] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
