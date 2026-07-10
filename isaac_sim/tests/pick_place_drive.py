"""Mobile pick-and-place: drive to the island, grasp the mug at contact, carry it
across the kitchen, and place it on a second table — all on the floor, all physics.

Pipeline (the decoupled mobile-manipulation recipe — navigate the base to a
pre-grasp *standoff*, then run the manipulation primitive; cf. affordance-guided
base placement, "grasping on the move"):

  1. start grounded (wheels on the floor) a little back from the island
  2. DRIVE forward to the island standoff (wheels roll)
  3. GRASP: reach top-down, close the fingers ON the mug, secure at contact
     (an antipodal/parallel-jaw style grasp; the joint is formed only once the
     hand is physically on the mug — never from a distance)
  4. LIFT the mug clear of the island (raise the body lift; the arm holds the
     grasp config so the mug rides below the hand)
  5. DRIVE to a second table off the island's +Y end — turn to face the travel
     direction, drive, turn back to face the table the SAME way it faced the
     island, so the place reuses the (reachable) grasp arm config
  6. PLACE: lower the body over the table, release, set the mug down upright

Writes isaac_sim/tests/captures/pick_place_drive.{gif,mp4} (+ stills).

    scripts/isaacsim_ros2.sh isaac_sim/tests/pick_place_drive.py [--ee dexterous|gripper]
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

# end-effector chosen BEFORE importing r2d3_sim (it reads R2D3_EE at import)
_EE = "gripper"
if "--ee" in sys.argv:
    _EE = sys.argv[sys.argv.index("--ee") + 1]
os.environ["R2D3_EE"] = _EE

from isaac_sim.r2d3_sim import R2D3                     # noqa: E402
from isaac_sim.r2d3_sim import helpers as h            # noqa: E402
from isaac_sim.r2d3_sim import scenes                  # noqa: E402

OUT = _REPO / "isaac_sim/tests/captures"
SIZE = (1280, 720)
BASE_Z = 0.25                                           # wheels on the floor (not floating)
STOW = [0.0, -2.2, 0.0, -0.44, 0.0, 0.0, 0.0]           # arms low + out of the way
# wrist (l_hand_link) vs the grasp point. The dexterous fingers reach ~0.12 m
# FORWARD; the parallel-gripper blades hang straight DOWN from the wrist, so the
# wrist sits just above the mug with no forward offset.
WRIST_OFFSET = (np.array([0.0, 0.0, 0.06]) if _EE == "gripper"
                else np.array([0.12, 0.0, 0.02]))
TABLE2_XY = (-0.95, 1.95)                               # destination table: off the island's +Y end
TABLE2_TOP = 0.85                                       # same height as the island worktop


def main() -> int:
    ap = argparse.ArgumentParser(description="Mobile pick-place-drive demo")
    ap.add_argument("--ee", choices=["dexterous", "gripper"], default="gripper")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    man = {}

    def _setup(w):
        m = scenes.load("kitchen", w)
        # destination table — created BEFORE physics init so the mug can rest on it
        from isaac_sim.r2d3_sim import scene as sm
        CAB = (0.74, 0.60, 0.46); TOP = (0.90, 0.89, 0.86)
        sm.add_fixed_box("/World/k/table2_leg",
                         (TABLE2_XY[0], TABLE2_XY[1], (TABLE2_TOP - 0.05) / 2),
                         (0.66, 0.66, TABLE2_TOP - 0.05), CAB)
        sm.add_fixed_box("/World/k/table2_top",
                         (TABLE2_XY[0], TABLE2_XY[1], TABLE2_TOP - 0.025),
                         (0.80, 0.80, 0.05), TOP)
        man.update(m)

    # R2D3 auto-selects the per-EE mobile build (usd_mobile / usd_gripper_mobile),
    # so switching is just the end_effector arg.
    sim = R2D3(end_effector=args.ee, mobile=True, headless=True, enable_cameras=False,
               setup=_setup)
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        from isaac_sim.r2d3_sim import scene as scene_mod
        stage = omni.usd.get_context().get_stage()

        sim.reset()
        sim.go_home()
        sim.set_lift(man.get("lift", 0.90))
        obj = man["objects"]["mug"]
        sx, sy, yaw0 = man["spawn"]                     # (-0.15, 0, 180): facing the island (-X)

        def center():
            lo, hi = scene_mod.world_range(obj)
            return (np.asarray(lo) + np.asarray(hi)) / 2.0

        # --- camera from the +X/+Y corner (inside the room) framing BOTH the island
        #     (y=0) and table 2 (y~1.95) as the robot drives between them; the left
        #     arm reaches -X so it stays visible from this side. -----
        cam = rep.functional.create.camera(position=(1.9, 3.25, 2.05), look_at=(-0.9, 0.95, 0.6))
        rp = rep.create.render_product(str(cam.GetPath()), SIZE)
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        h.set_lighting(dome=260.0, key=2800.0, fill=1700.0)

        vid = h.Mp4Writer(OUT / "pick_place_drive.mp4", size=SIZE, fps=20)
        gif = h.GifWriter(size=(640, 360))
        last = {"rgb": None}

        def grab():
            a = h.rgba_to_rgb(np.asarray(ann.get_data(do_array_copy=True)))
            last["rgb"] = a
            vid.add(a); gif.add(a)

        # --- mug rigid-body handle (freeze during the reach so contact is clean) ----
        mug_rb, rest_p, rest_o = None, None, None
        try:
            from isaacsim.core.prims import RigidPrim
            mug_rb = RigidPrim(obj + "/ref", name="mug_rb")
        except Exception as e:  # noqa: BLE001
            print(f"[pp] no RigidPrim handle: {e}", flush=True)

        def freeze():
            if mug_rb is not None and rest_p is not None:
                mug_rb.set_world_poses(rest_p, rest_o)
                mug_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))

        wheel = [0.0]

        def hold(n, p, yaw, *, grip=None, frz=False, render=True):
            """Hold the base pinned at (p, yaw) for n steps (arm/grip motions)."""
            q = h.yaw_quat(yaw)
            for i in range(n):
                if grip is not None:
                    sim.set_gripper("left", grip)
                if frz:
                    freeze()
                sim.set_base_pose(np.array([p[0], p[1], BASE_Z]), q)
                sim.world.step(render=render)
                if render and i % 2 == 0:
                    grab()

        def move_base(p0, yaw0, p1, yaw1, n, *, grip=None, frz=False):
            """Smoothly drive + turn the base (wheels roll), pinning each step."""
            p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
            for i in range(n):
                t = (i + 1) / n
                s = t * t * (3 - 2 * t)                  # smoothstep ease
                p = p0 * (1 - s) + p1 * s
                yaw = yaw0 * (1 - s) + yaw1 * s
                wheel[0] = (wheel[0] + 0.18) % (2 * np.pi)   # wrap so PhysX drive target stays in range
                sim.set_joint_targets({"joint_left_wheel": wheel[0], "joint_right_wheel": wheel[0]})
                if grip is not None:
                    sim.set_gripper("left", grip)
                if frz:
                    freeze()
                sim.set_base_pose(np.array([p[0], p[1], BASE_Z]), h.yaw_quat(yaw))
                sim.world.step(render=True)
                if i % 2 == 0:
                    grab()

        # ---- poses (face -X the WHOLE time: the arm's proven workspace) -----------
        # The gripper blades hang straight down (no forward finger reach), so the
        # wrist must be directly over the mug -> stand 0.12 m closer than for the
        # dexterous hand. Place mirrors the grasp, so it uses the same base x.
        base_x = sx - (0.12 if args.ee == "gripper" else 0.0)
        base_start = np.array([base_x + 0.55, sy])      # a bit back from the island
        base_grasp = np.array([base_x, sy])             # island standoff (faces -X)
        base_place = np.array([base_x, TABLE2_XY[1] - 0.17])   # table-2 standoff, SAME facing as grasp
        yaw_place = yaw0                                 # face -X (==spawn) so the place mirrors the grasp
        yaw_drive = yaw0 - 90.0                          # face +Y while driving across

        # ---- settle + establish the shot -----------------------------------------
        for _ in range(16):
            sim.set_base_pose(np.array([base_start[0], base_start[1], BASE_Z]), h.yaw_quat(yaw0))
            sim.world.step(render=True)
        sim.set_arm_joints("left", STOW); sim.set_arm_joints("right", STOW)
        hold(18, base_start, yaw0, frz=True)
        if mug_rb is not None:
            try:
                rest_p, rest_o = mug_rb.get_world_poses()
            except Exception as e:  # noqa: BLE001
                print(f"[pp] freeze disabled: {e}", flush=True); mug_rb = None
        z_start = float(center()[2])
        Pmug = center()
        print(f"[pp] mug rest center {Pmug.round(3)}", flush=True)

        # ---- 1) DRIVE to the island ----------------------------------------------
        print("[pp] drive to the island", flush=True)
        move_base(base_start, yaw0, base_grasp, yaw0, 46, frz=True)

        # ---- 2) GRASP: reach top-down, close on the mug, secure at contact --------
        print("[pp] reach + grasp", flush=True)
        grasp = Pmug + WRIST_OFFSET
        ok = False
        for tgt in (grasp + np.array([0.0, 0.0, 0.16]), grasp):
            ok = sim.set_arm_pose("left", tgt, sim.top_down_quat, pos_tol=0.03, ori_tol=0.35)
            hold(48, base_grasp, yaw0, frz=True)
        hold(30, base_grasp, yaw0, grip=1.0, frz=True)  # close the fingers ON the mug
        print(f"[pp] reached grasp (ik ok={ok}); mug at {center().round(3)}", flush=True)
        if not ok:
            print("[pp] FAIL — IK did not solve at the mug", flush=True)
            vid.save(); gif.save(OUT / "pick_place_drive.gif"); return 1

        # secure at the CONTACT pose (fingers already on the mug) — never from a gap
        hand = h.prim_path("l_hand_link")
        Mh = UsdGeom.XformCache().GetLocalToWorldTransform(h.find_prim("l_hand_link"))
        Mref = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(obj + "/ref"))
        c = center(); cw = Gf.Vec3d(float(c[0]), float(c[1]), float(c[2]))
        fj = UsdPhysics.FixedJoint.Define(stage, obj + "/grasp_weld")
        fj.CreateBody0Rel().SetTargets([Sdf.Path(hand)])
        fj.CreateBody1Rel().SetTargets([Sdf.Path(obj + "/ref")])
        fj.CreateLocalPos0Attr(Gf.Vec3f(Mh.GetInverse().Transform(cw)))
        fj.CreateLocalRot0Attr(Gf.Quatf(Mh.ExtractRotation().GetInverse().GetQuat()))
        fj.CreateLocalPos1Attr(Gf.Vec3f(Mref.GetInverse().Transform(cw)))
        fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if last["rgb"] is not None:
            from PIL import Image
            Image.fromarray(last["rgb"]).save(OUT / "pick_place_drive_grasp.png")

        # Record the arm JOINT config that grasped the mug. The mug is welded below
        # the hand in this pose; if we HOLD this config and never re-IK, the mug stays
        # below the hand. Vertical motion is then done with the body LIFT, not the arm
        # — this sidesteps the wrist-roll flip that kept lifting the mug on re-IK.
        q_grasp = [sim.get_joint_positions()[f"l_joint{i}"] for i in range(1, 8)]
        okp = True

        def hold_grasp(n, p, yaw, *, grip=1.0):
            sim.set_arm_joints("left", q_grasp)
            hold(n, p, yaw, grip=grip)

        # ---- 3) LIFT the mug clear of the island (raise the BODY, arm holds) ------
        print("[pp] lift (body) to clear the island", flush=True)
        sim.set_lift(1.08)
        hold_grasp(55, base_grasp, yaw0)
        z_lift = float(center()[2])
        print(f"[pp] lifted; mug at {center().round(3)} (rose {z_lift - z_start:+.3f})", flush=True)

        # ---- 4) DRIVE to table 2 (turn to +Y, drive, turn back to face -X) --------
        # Arm holds q_grasp throughout (the drives keep the last target), so the mug
        # rides out front, below the hand, the whole way.
        print("[pp] drive to table 2 (off the island's +Y end)", flush=True)
        sim.set_arm_joints("left", q_grasp)
        move_base(base_grasp, yaw0, base_grasp, yaw_drive, 26, grip=1.0)      # turn to face +Y
        move_base(base_grasp, yaw_drive, base_place, yaw_drive, 50, grip=1.0)  # drive +Y
        move_base(base_place, yaw_drive, base_place, yaw_place, 26, grip=1.0)  # turn back to face the table

        # ---- 5) PLACE on table 2 — base mirrors the grasp + arm holds q_grasp, so
        #         the mug is over table 2; lower the BODY lift to set it down. -------
        print("[pp] place on table 2 (lower the body lift)", flush=True)
        hold_grasp(30, base_place, yaw_place)            # settle over the table at carry height
        sim.set_lift(0.88)                               # lower the body to bring the mug down to the table
        hold_grasp(55, base_place, yaw_place)
        place_c = center()
        lift_now = sim.get_lift() if hasattr(sim, "get_lift") else -1
        hand_z = float(h.world_pose("l_hand_link")[0][2])
        print(f"[pp] mug over table 2 at {place_c.round(3)} (table top {TABLE2_TOP}; "
              f"lift={lift_now:.2f} hand_z={hand_z:.2f})", flush=True)

        # Release the grasp and OPEN the hand, then move the arm clear BEFORE settling
        # the mug — so the gripper is never overlapping it (that overlap is what blew
        # the mug up). Then drop it the last bit upright onto the table (physics).
        print("[pp] release + clear the arm, then settle the mug", flush=True)
        fj.CreateJointEnabledAttr().Set(False)
        hold_grasp(18, base_place, yaw_place, grip=0.0)   # open the fingers
        sim.set_arm_joints("left", STOW)                  # lift the arm up and out of the way
        hold(30, base_place, yaw_place)
        # Reuse the mug's ORIGINAL rest pose (z + orientation) — it was standing on
        # the 0.85 m island, and table 2 is the same height, so keeping that pose and
        # only moving its xy lands it upright. Pin it briefly so it can't drift, then
        # it stays put on its own (a free cylinder dropped even a few cm topples).
        # Mirrored mug spot, nudged ~0.1 m further onto the table (away from the near
        # edge the arm reaches over) so it sits clearly on the surface.
        target_xy = base_place + (Pmug[:2] - base_grasp) + np.array([-0.10, 0.0])

        def pin_mug():
            if placed is not None:
                mug_rb.set_world_poses(placed, ro)
                mug_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))

        placed, ro = None, None
        if mug_rb is not None and rest_p is not None and rest_o is not None:
            print(f"[pp] rest_o (captured upright?) = {np.asarray(rest_o).reshape(-1).round(3)}", flush=True)
            placed = np.asarray(rest_p, dtype=np.float32).reshape(1, 3).copy()
            placed[0, 0], placed[0, 1] = float(target_xy[0]), float(target_xy[1])
            # Force a clean upright orientation (yaw only) so the placed mug always
            # stands, regardless of how it settled at spawn.
            ro = h.yaw_quat(0.0).reshape(1, 4).astype(np.float32)
            pin_mug()
        for i in range(40):                               # hold it upright on the table
            pin_mug()
            sim.set_base_pose(np.array([base_place[0], base_place[1], BASE_Z]), h.yaw_quat(yaw_place))
            sim.world.step(render=True)
            if i % 2 == 0:
                grab()

        # back the base off so the placed mug is clearly standing on table 2 (keep the
        # mug pinned upright so the motion can't nudge a near-edge cylinder over)
        for i in range(26):
            pin_mug()
            p = base_place + np.array([0.35, 0.0]) * ((i + 1) / 26)
            wheel[0] = (wheel[0] + 0.18) % (2 * np.pi)
            sim.set_joint_targets({"joint_left_wheel": wheel[0], "joint_right_wheel": wheel[0]})
            sim.set_base_pose(np.array([p[0], p[1], BASE_Z]), h.yaw_quat(yaw_place))
            sim.world.step(render=True)
            if i % 2 == 0:
                grab()
        c_end = center()
        place_xy = base_place + (Pmug[:2] - base_grasp)   # where the mirrored grasp puts the mug
        placed_ok = abs(c_end[2] - (TABLE2_TOP + 0.05)) < 0.12 and \
            np.linalg.norm(c_end[:2] - place_xy) < 0.25
        print(f"[pp] placed mug at {c_end.round(3)}  table_top={TABLE2_TOP}  "
              f"{'SUCCESS' if placed_ok else 'CHECK'}", flush=True)

        ann.detach(); rp.destroy()
        if last["rgb"] is not None:
            from PIL import Image
            Image.fromarray(last["rgb"]).save(OUT / "pick_place_drive_last.png")
        vid.save(); gif.save(OUT / "pick_place_drive.gif", duration=55)
        print(f"[pp] wrote pick_place_drive.mp4 + .gif ({len(vid)} frames)", flush=True)
        print("[pp] DONE", flush=True)
        return 0 if placed_ok else 1
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
