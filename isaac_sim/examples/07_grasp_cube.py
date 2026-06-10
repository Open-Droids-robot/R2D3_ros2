"""IK grasp-and-lift of a red cube, using the platform SDK.

    scripts/isaacsim_ros2.sh isaac_sim/examples/07_grasp_cube.py

Pattern: add a table+cube via the R2D3 ``setup`` hook (before physics init),
IK the wrist over the cube, close the hand, attach the cube to the hand with a
fixed joint (a friction grasp of the small cube is unreliable — same weld
simplification the diagnostics use), then raise the body lift.

The cube is held frozen on the table during the approach so the weld is computed
from a clean, known pose (otherwise the closing fingers knock it and the fixed
joint snaps the bodies together with an impulse).
"""
import numpy as np
from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

TABLE_TOP = 0.44
CUBE = 0.04
CUBE_C = np.array([0.52, -0.21, TABLE_TOP + CUBE / 2])
FINGER_OFFSET = np.array([0.12, 0.0, -0.05])   # dex fingers converge here vs l_hand_link
_HOLDER = {}


def build_scene(world):
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.api.materials import PhysicsMaterial
    mat = PhysicsMaterial(prim_path="/World/m", static_friction=1.4, dynamic_friction=1.2)
    FixedCuboid(prim_path="/World/table", name="table", position=np.array([0.6, -0.2, TABLE_TOP / 2]),
                scale=np.array([0.4, 0.4, TABLE_TOP]), color=np.array([0.6, 0.45, 0.3]),
                physics_material=mat)
    _HOLDER["cube"] = DynamicCuboid(prim_path="/World/cube", name="cube", position=CUBE_C,
                                    scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.9, 0.12, 0.12]),
                                    mass=0.03, physics_material=mat)


def main() -> int:
    sim = R2D3(end_effector="dexterous", headless=True, setup=build_scene)
    try:
        import omni.usd
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        sim.reset()
        cube = _HOLDER["cube"]

        def freeze():
            """Re-pin the cube to its rest pose + zero velocity (it sits on the
            table; this keeps it there while the arm moves in)."""
            cube.set_world_pose(position=CUBE_C, orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))

        for _ in range(40):           # settle, frozen on the table
            freeze(); sim.step()
        z0 = float(cube.get_world_pose()[0][2])

        grasp = CUBE_C - FINGER_OFFSET
        pre = grasp + np.array([0.0, 0.0, 0.13])
        for tgt in (pre, grasp):
            ok = sim.set_arm_pose("left", tgt, sim.top_down_quat, pos_tol=0.015, ori_tol=0.2)
            for _ in range(70):
                freeze(); sim.step()
        sim.set_gripper("left", 1.0)          # curl the fingers
        for _ in range(40):
            freeze(); sim.step()
        print(f"[grasp] reached grasp pose (ik ok={ok})", flush=True)

        # hold these joint targets through the lift
        q_weld = [sim.get_joint_state().get(f"l_joint{i}") for i in range(1, 8)]

        # attach the cube to the hand where it rests (local poses, else the
        # FixedJoint snaps it to the wrist origin)
        freeze(); sim.step()
        stage = omni.usd.get_context().get_stage()
        hand = h.prim_path("l_hand_link")
        Mh = UsdGeom.XformCache().GetLocalToWorldTransform(h.find_prim("l_hand_link"))
        cpos = np.asarray(cube.get_world_pose()[0], float)
        lp = Mh.GetInverse().Transform(Gf.Vec3d(float(cpos[0]), float(cpos[1]), float(cpos[2])))
        lrot = Mh.ExtractRotation().GetInverse().GetQuat()
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/cube/grasp_weld")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(hand)])
        fj.CreateBody1Rel().SetTargets([Sdf.Path("/World/cube")])
        fj.CreateLocalPos0Attr(Gf.Vec3f(lp)); fj.CreateLocalRot0Attr(Gf.Quatf(lrot))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

        # lift the body, holding the grasp (the weld carries the cube up)
        for i in range(140):
            sim.set_arm_joints("left", q_weld)
            sim.set_gripper("left", 1.0)
            sim.set_lift(0.5 + 0.22 * (i + 1) / 140)
            sim.step()

        z1 = float(cube.get_world_pose()[0][2])
        print(f"[grasp] cube z {z0:.3f} -> {z1:.3f} (rose {z1 - z0:+.3f})  "
              f"{'SUCCESS' if z1 - z0 > 0.1 else 'FAIL'}", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
