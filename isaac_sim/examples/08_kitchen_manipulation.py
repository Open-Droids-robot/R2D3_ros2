"""Pick a mug off the kitchen island — the "start here" manipulation example.

    scripts/isaacsim_ros2.sh isaac_sim/examples/08_kitchen_manipulation.py

Loads the kitchen training scene (`scenes.load`), which puts a mug + utensils on a
collidable island and stands the robot at it. The robot IKs the wrist over the mug,
closes the hand, attaches the mug with a fixed joint (the proven weld from example
07 — a friction grasp of a small object is unreliable), and lifts it. Swap
`TARGET` for "bowl" / "soup_can" / "cracker_box" to grasp a different item.
"""
import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h
from isaac_sim.r2d3_sim import scenes

TARGET = "mug"
# Wrist (l_hand_link) offset from the grasp point for a top-down dex grasp: the
# robot faces -X (yaw 180) at the island, so the wrist sits +0.12 m toward the robot
# (and +0.05 above) the object — opposite sign to example 07's +X-facing robot.
WRIST_OFFSET = np.array([0.12, 0.0, 0.05])


def main() -> int:
    man = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True,
               setup=lambda w: man.update(scenes.load("kitchen", w)))
    try:
        import omni.usd
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        from isaac_sim.r2d3_sim import scene as scene_mod
        stage = omni.usd.get_context().get_stage()

        sim.reset()
        sim.go_home()
        sim.set_lift(man.get("lift", 0.90))            # arms clear of the island
        sx, sy, yaw = man["spawn"]
        q = h.yaw_quat(yaw)
        base_pos = np.array([sx, sy, 0.27])

        def step(n):                                   # hold the free base in place
            for _ in range(n):
                sim.set_base_pose(base_pos, q)
                sim.world.step(render=False)

        sim.set_base_pose(base_pos, q)
        step(80)                                       # place + settle objects

        obj = man["objects"][TARGET]                   # /World/objs/<target>

        def center():
            lo, hi = scene_mod.world_range(obj)
            return (np.asarray(lo) + np.asarray(hi)) / 2.0

        z0 = float(center()[2])
        c0 = center()
        print(f"[grasp08] {TARGET} at {c0.round(3)}", flush=True)

        # Top-down approach: pre-grasp above, then down to the grasp pose.
        grasp = c0 + WRIST_OFFSET
        ok = False
        for tgt in (grasp + np.array([0.0, 0.0, 0.14]), grasp):
            ok = sim.set_arm_pose("left", tgt, sim.top_down_quat, pos_tol=0.025, ori_tol=0.3)
            step(70)
        sim.set_gripper("left", 1.0)                   # curl the fingers
        step(40)
        print(f"[grasp08] reached grasp pose (ik ok={ok})", flush=True)
        if not ok:
            print("[grasp08] IK did not solve — target out of reach (lower the island or move the spawn).", flush=True)
            return 1

        # Weld the mug to the hand (both anchors at the mug's world centre -> no snap,
        # robust to the asset's internal origin offset).
        hand = h.prim_path("l_hand_link")
        Mh = UsdGeom.XformCache().GetLocalToWorldTransform(h.find_prim("l_hand_link"))
        Mref = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(obj + "/ref"))
        c = center()
        cw = Gf.Vec3d(float(c[0]), float(c[1]), float(c[2]))
        fj = UsdPhysics.FixedJoint.Define(stage, obj + "/grasp_weld")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(hand)])
        fj.CreateBody1Rel().SetTargets([Sdf.Path(obj + "/ref")])
        fj.CreateLocalPos0Attr(Gf.Vec3f(Mh.GetInverse().Transform(cw)))
        fj.CreateLocalRot0Attr(Gf.Quatf(Mh.ExtractRotation().GetInverse().GetQuat()))
        fj.CreateLocalPos1Attr(Gf.Vec3f(Mref.GetInverse().Transform(cw)))
        fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

        # Lift by IK-ing the wrist straight up (avoids the body-lift ceiling).
        ok2 = sim.set_arm_pose("left", grasp + np.array([0.0, 0.0, 0.22]), sim.top_down_quat,
                               pos_tol=0.04, ori_tol=0.35)
        for _ in range(140):
            sim.set_gripper("left", 1.0)
            sim.set_base_pose(base_pos, q)
            sim.world.step(render=False)

        z1 = float(center()[2])
        rose = z1 - z0
        print(f"[grasp08] {TARGET} z {z0:.3f} -> {z1:.3f} (rose {rose:+.3f}, lift-ik ok={ok2})  "
              f"{'SUCCESS' if rose > 0.08 else 'FAIL'}", flush=True)
        return 0 if rose > 0.08 else 1
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
