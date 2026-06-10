"""Probe the left gripper pose for candidate arm configs (FK, no IK).

For each candidate left-arm joint vector, settle and report l_hand_link's world
position + its world axes (so we know the approach direction and the finger
closing axis = local Y) + the world bbox/gap of the two finger links. Used to
pick a config whose gripper sits in open space at a graspable spot, so we can
spawn the cube there.
"""
from __future__ import annotations
import os, sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("PRIVACY_CONSENT", "Y")

POSES = {
    "a_sh_down":   [0.0, -0.8, 0.0, -0.5, 0.0, 0.5, 0.0],
    "b_reach":     [0.0, -1.0, 0.0, -1.2, 0.0, 0.7, 0.0],
    "c_fwd":       [0.0,  0.6, 0.0, -1.2, 0.0, 0.8, 0.0],
    "d_extend":    [0.0, -0.6, 0.0, -1.8, 0.0, 1.0, 0.0],
    "e_down_out":  [0.4, -1.0, 0.0, -0.8, 0.0, 0.6, 0.0],
    "f_low":       [0.0, -1.3, 0.0, -0.6, 0.0, 0.9, 0.0],
}


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from pxr import Usd, UsdGeom, Gf
        from isaacsim.core.api import World
        from isaac_sim.r2d3_sim import scene as scene_mod
        from isaac_sim.r2d3_sim.robot import Robot

        world = World(stage_units_in_meters=1.0)
        rpath = scene_mod.assemble(world)
        world.reset()
        robot = Robot(prim_path=rpath); robot.initialize()
        robot.go_home()
        for _ in range(120):
            world.step(render=False)
        stage = omni.usd.get_context().get_stage()
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

        def find(name):
            for p in stage.Traverse():
                if p.GetName() == name and p.GetTypeName() == "Xform":
                    return p.GetPath().pathString
            for p in stage.Traverse():
                if p.GetName() == name:
                    return p.GetPath().pathString
            return None
        HAND = find("l_hand_link"); FL = find("l_finger_left"); FR = find("l_finger_right")
        print(f"[gp] HAND={HAND}", flush=True)
        print(f"[gp] FL={FL}", flush=True)

        def world_pose(path):
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            o = np.array(m.Transform(Gf.Vec3d(0, 0, 0)))
            ax = lambda v: (np.array(m.Transform(Gf.Vec3d(*v))) - o)
            X = ax((1, 0, 0)); Y = ax((0, 1, 0)); Z = ax((0, 0, 1))
            return o, X / np.linalg.norm(X), Y / np.linalg.norm(Y), Z / np.linalg.norm(Z)

        def bbcenter(path):
            p = stage.GetPrimAtPath(path)
            if not p:
                return None
            r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
            if r.IsEmpty():
                return None
            mn, mx = r.GetMin(), r.GetMax()
            return np.array([(mn[i] + mx[i]) / 2 for i in range(3)])

        for name, q in POSES.items():
            robot.set_finger("left", 0.035)   # open
            for _ in range(80):
                robot.set_arm_targets("left", q)
                world.step(render=False)
            o, X, Y, Z = world_pose(HAND)
            cl = bbcenter(FL); cr = bbcenter(FR)
            gap = float(np.linalg.norm(cl - cr)) if cl is not None and cr is not None else -1
            grasp = (cl + cr) / 2 if cl is not None and cr is not None else o
            print(f"[gp] {name:11s} hand={o.round(3)} grasp={grasp.round(3)} "
                  f"gap={gap:.3f}", flush=True)
            print(f"[gp]   approach(+X)={X.round(2)} close(+Y)={Y.round(2)} up(+Z)={Z.round(2)}",
                  flush=True)
        print("[gp] DONE", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
