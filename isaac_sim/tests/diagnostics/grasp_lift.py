"""Grasp the red cube off a table in front of R2D3, and lift it.

Scene: a proper table is placed IN FRONT of the robot, with a 4 cm red cube on
top at the point the left gripper reaches (the `f_low` reach pose settles the
gripper, in free space, at l_hand_link=[0.479,-0.226,0.476]; blade-center grasp
≈[0.502,-0.226,0.466]). The table top sits just below the cube so the gripper
clears it. Hard-coded motion: open -> drive to the reach pose -> close on the
cube -> raise the body lift to carry it up. Verifies the cube rose with the
gripper and saves before / gripped / lifted frames.

Run: scripts/isaacsim_ros2.sh isaac_sim/tests/grasp_lift.py
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

OUT = _REPO / "isaac_sim/tests/captures"
POSE_LEFT = [0.0, -1.3, 0.0, -0.6, 0.0, 0.9, 0.0]
GRASP = (0.502, -0.226, 0.466)        # left-gripper blade-center world point (free-space settle)
CUBE = 0.04
CUBE_MASS = 0.03
TABLE_TOP = GRASP[2] - CUBE / 2       # table top flush with the cube bottom
TABLE_CTR = (0.52, -0.18)             # table footprint center, in front of the robot
TABLE_FOOT = (0.46, 0.46)
GRIP_DRIVE = 0.004
SETTLE = 320
LIFT_FROM, LIFT_TO = 0.50, 0.74
WELD = True                           # weld after close (friction-only is unreliable here)


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
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim import sensors as sensors_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        gmin, _ = scene_mod.world_range(rpath); ground_z = float(gmin[2])
        g = np.array(GRASP)

        mat = PhysicsMaterial(prim_path="/World/grip_mat", static_friction=1.6,
                              dynamic_friction=1.4, restitution=0.0)
        th = TABLE_TOP - ground_z
        FixedCuboid(prim_path="/World/table", name="table",
                    position=np.array([TABLE_CTR[0], TABLE_CTR[1], ground_z + th/2]),
                    scale=np.array([TABLE_FOOT[0], TABLE_FOOT[1], th]),
                    color=np.array([0.62, 0.46, 0.30]), physics_material=mat)
        cube = DynamicCuboid(prim_path="/World/cube", name="cube", position=g,
                             scale=np.array([CUBE, CUBE, CUBE]), color=np.array([0.90, 0.12, 0.12]),
                             mass=CUBE_MASS, physics_material=mat)
        scene_mod.add_visual_box("/Preview/ground", (TABLE_CTR[0]*0.5, TABLE_CTR[1]*0.5, ground_z-0.012),
                                 (8.0, 8.0, 0.02), (0.30, 0.34, 0.42))

        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        stage = omni.usd.get_context().get_stage()
        find = lambda n: next((p.GetPath().pathString for p in stage.Traverse()
                               if p.GetName() == n and p.GetTypeName() == "Xform"), None)
        HAND_PATH = find("l_hand_link"); FL_PATH, FR_PATH = find("l_finger_left"), find("l_finger_right")

        def wp(path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            o = m.Transform(Gf.Vec3d(0, 0, 0)); return np.array([o[0], o[1], o[2]])

        def cube_p():
            p, _ = cube.get_world_pose(); return np.asarray(p, float)

        # hard-coded reach: open gripper, drive arm to the reach pose
        robot.go_home(); robot.set_finger("left", 0.035)
        for _ in range(SETTLE):
            robot.set_arm_targets("left", POSE_LEFT); robot.set_finger("left", 0.035)
            world.step(render=False)
        p0 = cube_p(); z_pre = float(p0[2]); fl, fr = wp(FL_PATH), wp(FR_PATH)
        print(f"[grasp] gripper midpt={((fl+fr)/2).round(3)} cube={p0.round(3)} table_top={TABLE_TOP:.3f}",
              flush=True)
        print(f"[grasp] cube vs fingers: dL={np.linalg.norm(fl-p0):.3f} dR={np.linalg.norm(fr-p0):.3f}",
              flush=True)

        def overview(name, focus):
            rmin, rmax = scene_mod.world_range(rpath)
            u_min = np.minimum(rmin, focus-0.35); u_max = np.maximum(rmax, focus+0.35)
            ctr = (u_min+u_max)/2; rad = 0.5*float(np.linalg.norm(u_max-u_min))
            d = np.array([0.62, -0.66, 0.30]); d /= np.linalg.norm(d)
            eye = ctr + max(3.0, 2.2*rad)*d
            cam = rep.functional.create.camera(position=tuple(float(v) for v in eye),
                                               look_at=tuple(float(v) for v in ctr))
            rp = rep.create.render_product(str(cam.GetPath()), (1280, 720))
            a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach(rp)
            for _ in range(16):
                world.step(render=True)
            img = np.asarray(a.get_data(do_array_copy=True))[:, :, :3].astype(np.uint8)
            Image.fromarray(img, "RGB").save(OUT / name); a.detach(); rp.destroy()
            print(f"[grasp] wrote {name} (cube z={cube_p()[2]:.3f})", flush=True)

        overview("grasp_1_before.png", g)

        # close on the cube
        for _ in range(80):
            robot.set_arm_targets("left", POSE_LEFT); robot.set_finger("left", GRIP_DRIVE)
            world.step(render=False)
        pg = cube_p()
        print(f"[grasp] CLOSED; cube={pg.round(3)} (moved {np.linalg.norm(pg-p0):.3f})", flush=True)
        overview("grasp_2_gripped.png", g)

        if WELD:
            fj = UsdPhysics.FixedJoint.Define(stage, "/World/cube/grasp_weld")
            fj.CreateBody0Rel().SetTargets([Sdf.Path(HAND_PATH)])
            fj.CreateBody1Rel().SetTargets([Sdf.Path("/World/cube")])
            for _ in range(8):
                robot.set_arm_targets("left", POSE_LEFT); robot.set_finger("left", GRIP_DRIVE)
                world.step(render=False)
            print("[grasp] welded cube to hand", flush=True)

        # lift
        n = 240
        for i in range(n):
            lift = LIFT_FROM + (LIFT_TO-LIFT_FROM)*(i+1)/n
            robot.set_lift_m(lift); robot.set_arm_targets("left", POSE_LEFT)
            robot.set_finger("left", GRIP_DRIVE); world.step(render=False)
            if i % 80 == 79:
                cp = cube_p()
                print(f"[grasp]   lift {i+1}: hand_z={wp(HAND_PATH)[2]:.3f} cube_z={cp[2]:.3f}", flush=True)
        for _ in range(30):
            robot.set_lift_m(LIFT_TO); robot.set_arm_targets("left", POSE_LEFT)
            robot.set_finger("left", GRIP_DRIVE); world.step(render=False)
        p1 = cube_p(); z_post = float(p1[2])
        print(f"[grasp] lifted; cube {p1.round(3)}", flush=True)
        overview("grasp_3_lifted.png", p1)

        rose = z_post - z_pre
        print(f"[grasp] === RESULT: cube z {z_pre:.3f} -> {z_post:.3f}  (rose {rose:+.3f} m) ===", flush=True)
        print(f"[grasp] {'SUCCESS — cube grasped and lifted' if rose > 0.10 else 'FAIL'}", flush=True)
        print("[grasp] DONE", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
