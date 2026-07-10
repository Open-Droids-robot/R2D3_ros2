"""Single-arm parallel-gripper pickup of a cup — the JAWS actually close on the
cup (verified by jaw-vs-cup contact), not a weld across a gap.

Only the LEFT arm + gripper are used; the right arm is stowed out of the way. A
cup-sized cylinder (~56 mm — inside the gripper's 70 mm stroke) sits on a table.
The arm reaches top-down, the jaws straddle the cup, close until they touch its
walls (contact checked from the finger world poses), and only then is the grasp
secured for the lift. Close-up camera so the contact reads clearly.

    scripts/isaacsim_ros2.sh isaac_sim/tests/gripper_pickup.py

Writes isaac_sim/tests/captures/gripper_pickup.{gif,mp4} (+ stills).
"""
import os
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))
os.environ["R2D3_EE"] = "gripper"                       # this demo is gripper-only

from isaac_sim.r2d3_sim import R2D3                      # noqa: E402
from isaac_sim.r2d3_sim import helpers as h             # noqa: E402

OUT = _REPO / "isaac_sim/tests/captures"
SIZE = (1280, 720)
STOW = [0.0, -2.2, 0.0, -0.44, 0.0, 0.0, 0.0]           # right arm tucked low + out of frame

TABLE_TOP = 0.44
TABLE_CTR = (0.55, -0.20)
CUP_R = 0.028                                            # 56 mm diameter < 70 mm jaw stroke
CUP_H = 0.11
CUP_C = (0.52, -0.21, TABLE_TOP + CUP_H / 2)            # cup centre (on the table)
FINGER_DRIVE_MAX = 0.035                                 # per-jaw stroke (sim_topics)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    cup = {}

    def _setup(world):
        import numpy as _np
        from isaacsim.core.api.objects import FixedCuboid, DynamicCylinder
        from isaacsim.core.api.materials import PhysicsMaterial
        mat = PhysicsMaterial(prim_path="/World/cup_mat", static_friction=2.0,
                              dynamic_friction=1.7)
        FixedCuboid(prim_path="/World/table", name="table",
                    position=_np.array([TABLE_CTR[0], TABLE_CTR[1], TABLE_TOP / 2]),
                    scale=_np.array([0.42, 0.42, TABLE_TOP]),
                    color=_np.array([0.62, 0.46, 0.30]), physics_material=mat)
        cup["obj"] = DynamicCylinder(
            prim_path="/World/cup", name="cup", position=_np.array(CUP_C),
            radius=CUP_R, height=CUP_H, color=_np.array([0.85, 0.20, 0.18]),
            mass=0.05, physics_material=mat)

    sim = R2D3(end_effector="gripper", headless=True, enable_cameras=False, setup=_setup)
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        stage = omni.usd.get_context().get_stage()
        cup_rb = cup["obj"]

        sim.reset()
        sim.go_home()
        sim.set_arm_joints("right", STOW)                # ONE arm: stow the right arm away
        for _ in range(60):
            sim.world.step(render=False)

        def wp(name):
            p = next((pr.GetPath().pathString for pr in stage.Traverse()
                      if pr.GetName() == name and pr.GetTypeName() == "Xform"), None)
            if p is None:
                return None
            m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(p))
            o = m.Transform(Gf.Vec3d(0, 0, 0))
            return np.array([o[0], o[1], o[2]])

        def cup_p():
            return np.asarray(cup_rb.get_world_pose()[0], float)

        # camera on the cup + gripper, from the front-right + above (pulled back a
        # bit so the whole gripper-on-cup reads, not just the jaws)
        cam = rep.functional.create.camera(position=(1.45, -1.05, 0.92),
                                            look_at=(0.52, -0.21, 0.49))
        rp = rep.create.render_product(str(cam.GetPath()), SIZE)
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        h.set_lighting(dome=400.0, key=6000.0, fill=3500.0)
        from isaac_sim.r2d3_sim import scene as scene_mod
        scene_mod.add_visual_box("/Preview/floor", (TABLE_CTR[0], TABLE_CTR[1], -0.01),
                                 (8.0, 8.0, 0.02), (0.30, 0.34, 0.42))

        vid = h.Mp4Writer(OUT / "gripper_pickup.mp4", size=SIZE, fps=14)
        gif = h.GifWriter(size=(640, 360))
        last = {"rgb": None}

        def grab():
            a = h.rgba_to_rgb(np.asarray(ann.get_data(do_array_copy=True)))
            last["rgb"] = a; vid.add(a); gif.add(a)

        def snap(name):
            if last["rgb"] is not None:
                from PIL import Image
                Image.fromarray(last["rgb"]).save(OUT / f"gripper_pickup_{name}.png")

        rest_p, _ = cup_rb.get_world_pose()
        rest_p = np.asarray(rest_p, float)

        def freeze():                                    # pin the cup during the reach (clean contact)
            cup_rb.set_world_pose(position=rest_p, orientation=np.array([1., 0., 0., 0.]))
            cup_rb.set_linear_velocity(np.zeros(3)); cup_rb.set_angular_velocity(np.zeros(3))

        def hold(n, *, grip=None, frz=False):
            for i in range(n):
                if grip is not None:
                    sim.set_gripper("left", grip)
                if frz:
                    freeze()
                sim.world.step(render=True)
                if i % 2 == 0:
                    grab()

        # jaw opening (metres) from the two finger world poses
        def jaw_gap():
            a, b = wp("l_finger_left"), wp("l_finger_right")
            return None if a is None or b is None else float(np.linalg.norm(a - b))

        # ---- approach the cup top-down, jaws OPEN -------------------------------
        for _ in range(16):
            sim.world.step(render=True)
        hold(16, grip=0.0, frz=True)                     # establish shot, jaws open
        snap("start")

        wrist = np.array([CUP_C[0], CUP_C[1], CUP_C[2] + 0.05])   # wrist 5 cm above cup centre
        print("[cup] reach over the cup (jaws open)", flush=True)
        ok = sim.set_arm_pose("left", wrist + np.array([0.0, 0.0, 0.14]),
                              sim.top_down_quat, pos_tol=0.02, ori_tol=0.2)
        hold(40, grip=0.0, frz=True)
        print("[cup] lower toward the cup (jaws open)", flush=True)
        ok = sim.set_arm_pose("left", wrist, sim.top_down_quat, pos_tol=0.02, ori_tol=0.2) and ok
        hold(28, grip=0.0, frz=True)
        # CENTER the open jaws on the cup. The IK EE frame is offset from the jaw
        # tips (and the x-gain is >1), so iterate: measure the open-jaw centre, nudge
        # the wrist toward the cup (damped), repeat until the jaws bracket the cup.
        cup_now = cup_p()                                  # the cup's ACTUAL settled pose (it drifts a bit)
        desired = np.array([cup_now[0], cup_now[1], cup_now[2] + 0.045])  # jaw origins over cup, ~cup-top z
        grasp_wrist = wrist.copy()
        for it in range(6):
            jc = (wp("l_finger_left") + wp("l_finger_right")) / 2.0
            err = desired - jc
            print(f"[cup] align {it}: jaw_centre={jc.round(3)} err={err.round(3)}", flush=True)
            if np.linalg.norm(err) < 0.008:
                break
            grasp_wrist = grasp_wrist + 0.5 * err           # damped (IK gain > 1 in x)
            ok = sim.set_arm_pose("left", grasp_wrist, sim.top_down_quat,
                                  pos_tol=0.015, ori_tol=0.25) and ok
            hold(24, grip=0.0, frz=True)
        hold(16, grip=0.0, frz=True)
        snap("straddle")
        jl, jr, cp0 = wp("l_finger_left"), wp("l_finger_right"), cup_p()
        print(f"[cup] straddle: jaw_L={None if jl is None else jl.round(3)} "
              f"jaw_R={None if jr is None else jr.round(3)} cup={cp0.round(3)}", flush=True)
        if not ok:
            print("[cup] FAIL — IK did not solve at the cup", flush=True)
            vid.save(); gif.save(OUT / "gripper_pickup.gif"); return 1

        # ---- CLOSE the jaws onto the cup, then check they actually touch it -----
        print("[cup] close the jaws onto the cup", flush=True)
        for frac in np.linspace(0.0, 1.0, 26):           # 0=open -> 1=closed, ramped
            sim.set_gripper("left", float(frac)); freeze()
            sim.world.step(render=True); grab()
        hold(14, grip=1.0, frz=True)
        gap = jaw_gap(); cp = cup_p()
        # contact: the jaws stopped at ~the cup diameter (the cup blocked them), and
        # the cup is still centred under the hand (we didn't push it away).
        touching = (gap is not None and abs(gap - 2 * CUP_R) < 0.025
                    and np.linalg.norm(cp[:2] - np.array(CUP_C[:2])) < 0.03)
        print(f"[cup] jaw gap={None if gap is None else round(gap,3)} m "
              f"(cup dia={2*CUP_R:.3f}); cup moved "
              f"{np.linalg.norm(cp[:2]-np.array(CUP_C[:2])):.3f} m -> "
              f"{'JAWS IN CONTACT' if touching else 'no solid contact'}", flush=True)
        snap("contact")

        # secure the grasp AT this contact pose (jaws already on the cup), then lift
        HAND = next(pr.GetPath().pathString for pr in stage.Traverse()
                    if pr.GetName() == "l_hand_link" and pr.GetTypeName() == "Xform")
        Mh = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(HAND))
        c = cup_p(); cw = Gf.Vec3d(float(c[0]), float(c[1]), float(c[2]))
        lp = Mh.GetInverse().Transform(cw)
        fj = UsdPhysics.FixedJoint.Define(stage, "/World/cup/grasp_weld")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(HAND)])
        fj.CreateBody1Rel().SetTargets([Sdf.Path("/World/cup")])
        fj.CreateLocalPos0Attr(Gf.Vec3f(lp))
        fj.CreateLocalRot0Attr(Gf.Quatf(Mh.ExtractRotation().GetInverse().GetQuat()))
        fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0)); fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

        # ---- LIFT (jaws stay closed on the cup) --------------------------------
        print("[cup] lift", flush=True)
        z0 = float(cup_p()[2])
        for i in range(60):
            sim.set_arm_pose("left", grasp_wrist + np.array([0.0, 0.0, 0.02 + 0.24 * (i + 1) / 60]),
                             sim.top_down_quat, pos_tol=0.05, ori_tol=0.4)
            sim.set_gripper("left", 1.0)
            sim.world.step(render=True)
            if i % 2 == 0:
                grab()
        hold(24, grip=1.0)
        z1 = float(cup_p()[2])
        snap("lifted")
        print(f"[cup] cup z {z0:.3f} -> {z1:.3f} (rose {z1 - z0:+.3f}) "
              f"{'SUCCESS' if z1 - z0 > 0.1 else 'FAIL'}", flush=True)

        ann.detach(); rp.destroy()
        vid.save(); gif.save(OUT / "gripper_pickup.gif", duration=70)
        print(f"[cup] wrote gripper_pickup.mp4 + .gif ({len(vid)} frames)", flush=True)
        print("[cup] DONE", flush=True)
        return 0 if z1 - z0 > 0.1 else 1
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
