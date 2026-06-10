"""Fast (no-render) diagnostic of the IK top-down grasp geometry.

Reaches the grasp pose via IK and prints hand + finger world positions vs the
cube at each phase, so we can see why the close misses.
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
TABLE_TOP = 0.46
CUBE = 0.03
CUBE_C = (0.50, -0.22, TABLE_TOP + CUBE/2)
GRASP_Z = TABLE_TOP + 0.05      # hand a blade-length above the table -> blades straddle the cube, hand clears its top
PREGRASP_Z = 0.62


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
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        gmin, _ = scene_mod.world_range(rpath); ground_z = float(gmin[2])
        mat = PhysicsMaterial(prim_path="/World/m", static_friction=1.6, dynamic_friction=1.4)
        th = TABLE_TOP - ground_z
        FixedCuboid(prim_path="/World/table", name="t",
                    position=np.array([0.50, -0.18, ground_z+th/2]), scale=np.array([0.40,0.40,th]),
                    color=np.array([0.6,0.45,0.3]), physics_material=mat)
        cube = DynamicCuboid(prim_path="/World/cube", name="c", position=np.array(CUBE_C),
                             scale=np.array([CUBE,CUBE,CUBE]), color=np.array([0.9,0.1,0.1]),
                             mass=0.03, physics_material=mat)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        stage = omni.usd.get_context().get_stage()
        f = lambda n: next((p.GetPath().pathString for p in stage.Traverse()
                            if p.GetName()==n and p.GetTypeName()=="Xform"), None)
        HAND, FL, FR, BASE = f("l_hand_link"), f("l_finger_left"), f("l_finger_right"), f("base_link_underpan")

        def wp(path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            o = m.Transform(Gf.Vec3d(0,0,0)); return np.array([o[0],o[1],o[2]])
        def cube_p():
            p,_ = cube.get_world_pose(); return np.asarray(p,float)
        def report(tag):
            h,fl,fr,c = wp(HAND), wp(FL), wp(FR), cube_p()
            print(f"[g] {tag:10s} hand={h.round(3)} fL={fl.round(3)} fR={fr.round(3)} "
                  f"finger-gap={np.linalg.norm(fl-fr):.3f} cube={c.round(3)} "
                  f"dL={np.linalg.norm(fl-c):.3f} dR={np.linalg.norm(fr-c):.3f}", flush=True)

        bo = wp(BASE)
        ik = LulaKinematicsSolver(robot_description_path=DESC, urdf_path=URDF)
        ik.set_robot_base_pose(bo, np.array([1.,0.,0.,0.]))
        quat = mat_to_quat(np.array([[0,0,1],[0,1,0],[-1,0,0]],float))
        qp,_ = ik.compute_inverse_kinematics(EE, np.array([CUBE_C[0],CUBE_C[1],PREGRASP_Z]), quat, position_tolerance=0.01, orientation_tolerance=0.05)
        qg,_ = ik.compute_inverse_kinematics(EE, np.array([CUBE_C[0],CUBE_C[1],GRASP_Z]), quat, position_tolerance=0.01, orientation_tolerance=0.05)

        robot.go_home(); robot.set_finger("left", 0.035)
        for _ in range(60):
            world.step(render=False)
        for _ in range(120):
            robot.set_arm_targets("left", list(qp)); robot.set_finger("left", 0.035); world.step(render=False)
        report("pregrasp")
        for _ in range(120):
            robot.set_arm_targets("left", list(qg)); robot.set_finger("left", 0.035); world.step(render=False)
        report("grasp")
        for _ in range(90):
            robot.set_arm_targets("left", list(qg)); robot.set_finger("left", 0.004); world.step(render=False)
        report("closed")
        for i in range(120):
            robot.set_lift_m(0.5+0.2*(i+1)/120); robot.set_arm_targets("left", list(qg)); robot.set_finger("left", 0.004); world.step(render=False)
        report("lifted")
        print("[g] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
