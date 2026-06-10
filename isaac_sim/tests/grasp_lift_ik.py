"""IK (Lula) grasp-and-lift, switchable between the 5-finger DEXTEROUS hand and
the 2-finger parallel GRIPPER.

R2D3 reaches over the red cube on a table in front, closes the end-effector
around it, and lifts it. LulaKinematicsSolver positions the wrist; collision
stays on (cube rests on the table), the cube is attached to the wrist the moment
a fingertip/blade reaches it (before the stiff drive can knock it), the EE
closes (set_finger), and the body lift raises it. Captures a third-person GIF.

Run:
  scripts/isaacsim_ros2.sh isaac_sim/tests/grasp_lift_ik.py --ee dexterous
  scripts/isaacsim_ros2.sh isaac_sim/tests/grasp_lift_ik.py --ee gripper
(needs the matching USD built: scripts/build_robot.sh both)
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

# --- end-effector selection: parse --ee BEFORE importing r2d3_sim (it reads
#     R2D3_EE at import to pick the USD + joints). ---
def _arg_ee() -> str:
    a = sys.argv
    if "--ee" in a:
        try:
            return a[a.index("--ee") + 1].strip().lower()
        except IndexError:
            pass
    return os.environ.get("R2D3_EE", "dexterous").strip().lower()
EE_KIND = _arg_ee()
if EE_KIND not in ("dexterous", "gripper"):
    EE_KIND = "dexterous"
os.environ["R2D3_EE"] = EE_KIND

OUT = _REPO / "isaac_sim/tests/captures"
EE_FRAME = "l_hand_link"        # Lula IK target frame (the arm hand link)
WELD_LINK = "l_hand_link"
TABLE_TOP = 0.44
TABLE_CTR = (0.62, -0.18)
TABLE_FOOT = (0.42, 0.42)
CUBE = 0.04
# cube placed so the wrist reaches it for BOTH end-effectors: the gripper wrist
# sits right over the cube (x~0.52), the dex wrist ~0.12 m behind (x~0.40).
CUBE_C = (0.52, -0.21, TABLE_TOP + CUBE / 2)

# Per-end-effector config. finger_offset = where the grasp point sits relative
# to l_hand_link (world, top-down): the dex fingers converge ~0.12 m forward;
# the short gripper blades grasp just below the wrist. tip = the link gated on
# to trigger the attach.
_CFG = {
    "dexterous": dict(urdf="r2d3_v1_dexterous.urdf", desc="r2d3_left_arm_lula.yaml",
                      finger_offset=(0.12, 0.0, -0.05), tip="l_dex_index_tip",
                      weld_thresh=0.045, gif="grasp_dexterous.gif"),
    "gripper":   dict(urdf="r2d3_v1_gripper.urdf", desc="r2d3_left_arm_lula_gripper.yaml",
                      finger_offset=(0.0, 0.0, -0.05), tip="l_finger_left",
                      weld_thresh=0.055, gif="grasp_gripper.gif"),
}[EE_KIND]
URDF = str(_REPO / "isaac_sim/urdf" / _CFG["urdf"])
DESC = str(_REPO / "isaac_sim/urdf" / _CFG["desc"])
FINGER_OFFSET = _CFG["finger_offset"]
GRASP = (CUBE_C[0] - FINGER_OFFSET[0], CUBE_C[1] - FINGER_OFFSET[1], CUBE_C[2] - FINGER_OFFSET[2])
PREGRASP = (GRASP[0], GRASP[1], GRASP[2] + 0.13)


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


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        gmin, _ = scene_mod.world_range(rpath); ground_z = float(gmin[2])
        mat = PhysicsMaterial(prim_path="/World/m", static_friction=1.4, dynamic_friction=1.2)
        th = TABLE_TOP - ground_z
        FixedCuboid(prim_path="/World/table", name="table",
                    position=np.array([TABLE_CTR[0], TABLE_CTR[1], ground_z+th/2]),
                    scale=np.array([TABLE_FOOT[0], TABLE_FOOT[1], th]),
                    color=np.array([0.62, 0.46, 0.30]), physics_material=mat)
        cube = DynamicCuboid(prim_path="/World/cube", name="cube", position=np.array(CUBE_C),
                             scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]),
                             mass=0.03, physics_material=mat)
        scene_mod.add_visual_box("/Preview/ground", (TABLE_CTR[0]*0.5, TABLE_CTR[1]*0.5, ground_z-0.012),
                                 (8.0, 8.0, 0.02), (0.30, 0.34, 0.42))

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        scene_mod._hide_legacy_hand_flanges()    # post-reset: payload is loaded now
        stage = omni.usd.get_context().get_stage()
        f = lambda n: next((p.GetPath().pathString for p in stage.Traverse()
                            if p.GetName()==n and p.GetTypeName()=="Xform"), None)
        HAND, BASE = f(WELD_LINK), f("base_link_underpan")
        def wp(path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            o = m.Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])
        def cube_p():
            p,_ = cube.get_world_pose(); return np.asarray(p,float)

        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        ik.set_robot_base_pose(wp(BASE), np.array([1.,0.,0.,0.]))
        quat = mat_to_quat(np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
        q_pre,ok1 = ik.compute_inverse_kinematics(EE_FRAME, np.array(PREGRASP), quat, position_tolerance=0.012, orientation_tolerance=0.15)
        q_gr,ok2  = ik.compute_inverse_kinematics(EE_FRAME, np.array(GRASP), quat, position_tolerance=0.012, orientation_tolerance=0.15)
        print(f"[grasp] IK pre={ok1} grasp={ok2}  grasp_target={np.array(GRASP).round(3)}", flush=True)
        if not (ok1 and ok2):
            return 1
        q_pre, q_gr = np.array(q_pre), np.array(q_gr)

        # clean third-person camera on the workspace
        ctr = np.array([0.56, -0.16, 0.56]); d = np.array([0.74, -0.52, 0.30]); d/=np.linalg.norm(d)
        cam = rep.functional.create.camera(position=tuple(float(v) for v in (ctr+2.0*d)), look_at=tuple(float(v) for v in ctr))
        rp = rep.create.render_product(str(cam.GetPath()), (960,540))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        frames=[]
        def grab():
            a=np.asarray(ann.get_data(do_array_copy=True))
            if a.ndim==3 and a.shape[2]==4: a=a[:,:,:3]
            frames.append(Image.fromarray(a.astype(np.uint8)).resize((480,270)))
        rest = np.array(CUBE_C)
        def freeze():
            cube.set_world_pose(position=rest, orientation=np.array([1.,0.,0.,0.]))
            cube.set_linear_velocity(np.zeros(3)); cube.set_angular_velocity(np.zeros(3))
        def move(q0,q1,n,finger,frz=False,cap=6):
            q0=np.array(q0); q1=np.array(q1)
            for i in range(n):
                a=(i+1)/n
                robot.set_arm_targets("left", list(q0+(q1-q0)*a)); robot.set_finger("left", finger)
                if frz: freeze()
                world.step(render=True)
                if i%cap==0: grab()

        # collision stays ON the whole time. Let the cube settle on the table.
        robot.go_home(); robot.set_finger("left", 0.035)
        for i in range(90):
            world.step(render=True)
            if i%14==0: grab()
        q_home = np.array([robot.get_joint_positions()[f"l_joint{i}"] for i in range(1,8)])
        z_pre = float(cube_p()[2])
        TIP = f(_CFG["tip"])

        print("[grasp] reach over cube", flush=True)
        move(q_home, q_pre, 70, 0.035)
        print("[grasp] lower in until a fingertip reaches the cube, then attach", flush=True)
        q_weld = np.array(q_gr)
        for i in range(90):
            a = (i+1)/90
            q_weld = np.array(q_pre) + (np.array(q_gr)-np.array(q_pre))*a
            robot.set_arm_targets("left", list(q_weld)); robot.set_finger("left", 0.035)
            world.step(render=True)
            if i%5==0: grab()
            if np.linalg.norm(wp(TIP) - cube_p()) < _CFG["weld_thresh"]:   # fingertip/blade at the cube -> stop & attach
                break
        # weld the cube to the hand WHERE IT RESTS (local poses, else it snaps to
        # the wrist origin). No impact: the cube is still sitting on the table.
        Mh = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(HAND))
        Mh_inv = Mh.GetInverse(); cpos = cube_p()
        lp = Mh_inv.Transform(Gf.Vec3d(float(cpos[0]), float(cpos[1]), float(cpos[2])))
        lrot = Mh.ExtractRotation().GetInverse().GetQuat()
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/cube/grasp_weld")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(HAND)]); fj.CreateBody1Rel().SetTargets([Sdf.Path("/World/cube")])
        fj.CreateLocalPos0Attr(Gf.Vec3f(lp)); fj.CreateLocalRot0Attr(Gf.Quatf(lrot))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        print("[grasp] curl fingers around the cube", flush=True)
        for i in range(70):
            robot.set_arm_targets("left", list(q_weld)); robot.set_finger("left", 0.0)
            world.step(render=True)
            if i%6==0: grab()
        print(f"[grasp] attached; hand={wp(HAND).round(3)} cube={cube_p().round(3)}", flush=True)
        print("[grasp] lift", flush=True)
        n=150
        for i in range(n):
            robot.set_lift_m(0.5+0.22*(i+1)/n); robot.set_arm_targets("left", list(q_weld)); robot.set_finger("left", 0.0)
            world.step(render=True)
            if i%7==0: grab()
        for i in range(30):
            robot.set_lift_m(0.72); robot.set_arm_targets("left", list(q_weld)); robot.set_finger("left", 0.0)
            world.step(render=True)
            if i%7==0: grab()
        z_post=float(cube_p()[2])
        print(f"[grasp] === cube z {z_pre:.3f} -> {z_post:.3f} (rose {z_post-z_pre:+.3f}) ===", flush=True)
        print(f"[grasp] {'SUCCESS' if z_post-z_pre>0.1 else 'FAIL'}", flush=True)

        ann.detach(); rp.destroy()
        if frames:
            frames[0].save(OUT/_CFG["gif"], save_all=True, append_images=frames[1:], duration=90, loop=0)
            stem=_CFG["gif"][:-4]; frames[0].save(OUT/f"{stem}_first.png"); frames[-1].save(OUT/f"{stem}_last.png")
            print(f"[grasp] wrote {_CFG['gif']} ({len(frames)} frames)", flush=True)
        print("[grasp] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
