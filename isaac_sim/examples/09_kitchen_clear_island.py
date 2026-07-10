"""Clear the island — an ML-perception-driven pick-and-place.

    scripts/isaacsim_ros2.sh isaac_sim/examples/09_kitchen_clear_island.py

The robot does NOT know where the mug is. It looks at the kitchen island with its
head camera, runs an **open-vocabulary detector** (OWL-ViT, via
``r2d3_sim.perception``) on the RGB frame to localise "a red mug", unprojects that
pixel to a 3D world point using the depth image + camera intrinsics, then drives
up, grasps the mug at the perceived location (IK + fixed-joint weld, from examples
07/08), and places it in a drop zone — clearing it off the island.

This closes the full perceive -> ground -> act loop: a real vision model decides
*what* and *where*, the depth camera grounds it in 3D, and the arm acts. Outputs a
third-person GIF (set CLEAR_GIF=0 to skip) + the annotated head-cam frame.

Needs ``pip install transformers`` (OWL-ViT weights auto-download, ~600 MB, cached).
"""
import os
from pathlib import Path

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h
from isaac_sim.r2d3_sim import scenes
from isaac_sim.r2d3_sim import perception

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"
QUERIES = ["a red mug", "a bowl", "a cracker box", "a soup can", "a mustard bottle"]
TARGET = "a red mug"
WRIST_OFFSET = np.array([0.12, 0.0, 0.02])           # wrist vs grasp point; low z -> hand comes onto the mug
STOW_LEFT = [0.0, -2.2, 0.0, -0.44, 0.0, 0.0, 0.0]   # FK-found: left hand low + out of the head-cam cone
STOW_RIGHT = [0.0, -2.2, 0.0, -0.44, 0.0, 0.0, 0.0]  # tuck the (unused) right arm down, not idle-raised
BASE_Z = 0.25                                        # wheels on the floor (not floating)
GIF = os.environ.get("CLEAR_GIF", "1") == "1"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    man = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True, enable_cameras=True,
               camera_resolution=(1280, 960),
               setup=lambda w: man.update(scenes.load("kitchen", w)))
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdGeom, Gf, UsdPhysics, Sdf
        from PIL import Image
        from isaac_sim.r2d3_sim import scene as scene_mod
        stage = omni.usd.get_context().get_stage()

        sim.reset()
        sim.go_home()
        sim.set_lift(man.get("lift", 0.90))
        sx, sy, yaw = man["spawn"]
        q = h.yaw_quat(yaw)
        base_grasp = np.array([sx, sy, BASE_Z])
        base_obs = np.array([sx + 0.35, sy, BASE_Z])    # 0.35 m back: island enters the clear view cone
        obj = man["objects"]["mug"]

        def center():
            lo, hi = scene_mod.world_range(obj)
            return (np.asarray(lo) + np.asarray(hi)) / 2.0

        # third-person GIF camera from the robot's +Y (left) side so the LEFT arm doing
        # the grasp is visible, not hidden behind the torso.
        cam = rep.functional.create.camera(position=(2.1, 2.5, 1.55), look_at=(-0.60, 0.12, 0.96))
        rp = rep.create.render_product(str(cam.GetPath()), (1280, 720))
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)
        frames = []
        vid = h.Mp4Writer(OUT / "clear_island.mp4", size=(1280, 720), fps=14) if GIF else None

        def grab():
            a = h.rgba_to_rgb(np.asarray(ann.get_data(do_array_copy=True)))
            if vid is not None:
                vid.add(a)                               # full-res 720p -> clean mp4
            frames.append(Image.fromarray(a).resize((640, 360)))

        wheel = [0.0]

        def drive(b0, b1, n, *, frz=False):              # smooth wheeled base move (not a teleport)
            for i in range(n):
                t = (i + 1) / n
                b = b0 * (1.0 - t) + b1 * t
                wheel[0] += 0.22
                sim.set_joint_targets({"joint_left_wheel": wheel[0], "joint_right_wheel": wheel[0]})
                if frz:
                    freeze()
                sim.set_base_pose(b, q)
                sim.world.step(render=GIF)
                if GIF and i % 2 == 0:
                    grab()

        # freeze handle: pin the mug to its rest pose during the reach so the weld is
        # computed from a clean, static pose (else contact knocks it and the weld snaps)
        mug_rb, rest_p, rest_o = None, None, None
        try:
            from isaacsim.core.prims import RigidPrim
            mug_rb = RigidPrim(obj + "/ref", name="mug_rb")
        except Exception as e:  # noqa: BLE001
            print(f"[clear] (no RigidPrim freeze handle: {e})", flush=True)

        def freeze():
            if mug_rb is not None and rest_p is not None:
                mug_rb.set_world_poses(rest_p, rest_o)
                mug_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))

        def step(n, base, *, grip=None, frz=False, render=GIF):
            for i in range(n):
                if grip is not None:
                    sim.set_gripper("left", grip)
                if frz:
                    freeze()
                sim.set_base_pose(base, q)
                sim.world.step(render=render)
                if render and GIF and i % 3 == 0:
                    grab()            # one capture path -> feeds both the gif AND the mp4

        for _ in range(16):                             # RTX warm-up + initial settle (start at the observe pose)
            sim.set_base_pose(base_obs, q); sim.world.step(render=True)
        step(30, base_obs)
        if mug_rb is not None:
            try:
                rest_p, rest_o = mug_rb.get_world_poses()
            except Exception as e:  # noqa: BLE001
                print(f"[clear] (freeze disabled: {e})", flush=True); mug_rb = None
        z_start = float(center()[2])
        print(f"[clear] mug rest center {center().round(3)}", flush=True)

        # ---------- OBSERVE: stow both arms, point the head down, look at the island ----------
        sim.set_arm_joints("left", STOW_LEFT)
        sim.set_arm_joints("right", STOW_RIGHT)
        sim.set_head(0.0, -0.62)
        h.set_lighting(dome=400.0, key=4000.0, fill=2000.0)
        step(25, base_obs, frz=True, render=False)      # settle physics (fast)
        for i in range(18):                             # REFRESH the head render + show the "looking" in the GIF
            freeze(); sim.set_base_pose(base_obs, q); sim.world.step(render=True)
            if GIF and i % 2 == 0:
                grab()
        rgb, depth = sim.get_image("head", depth=True)
        H, W = depth.shape

        cam_prim = stage.GetPrimAtPath(sim.cameras._prims["head"])
        ucam = UsdGeom.Camera(cam_prim)
        fl = ucam.GetFocalLengthAttr().Get()
        fx = fl / ucam.GetHorizontalApertureAttr().Get() * W
        fy = fl / ucam.GetVerticalApertureAttr().Get() * H
        cx, cy = W / 2.0, H / 2.0
        M = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
        cam_pos = np.array(M.ExtractTranslation())
        Rm = M.ExtractRotationMatrix()

        def pixel_to_world(u, v):
            d = float(depth[int(round(v)), int(round(u))])
            if d <= 0:
                return None
            ray = Gf.Vec3d((u - cx) / fx, -(v - cy) / fy, -1.0).GetNormalized()
            wd = ray * Rm
            return cam_pos + d * np.array([wd[0], wd[1], wd[2]])

        # ---------- PERCEIVE: open-vocabulary detection -> 3D grasp point ----------
        dets = perception.detect(rgb, QUERIES, threshold=0.05)
        print(f"[clear] OWL-ViT: {len(dets)} dets; top {[(d[0], round(d[1], 2)) for d in dets[:4]]}", flush=True)
        mug = next((d for d in dets if d[0] == TARGET), None)
        if mug is None:
            print(f"[clear] FAIL — detector did not find {TARGET!r}", flush=True)
            return 1
        bx = mug[2]
        u, v = (bx[0] + bx[2]) / 2.0, (bx[1] + bx[3]) / 2.0
        Pmug = pixel_to_world(u, v)
        if Pmug is None:
            print("[clear] FAIL — no depth at the mug pixel", flush=True)
            return 1
        print(f"[clear] perceived {TARGET!r} ({mug[1]:.2f}) px({u:.0f},{v:.0f}) -> world {Pmug.round(3)} "
              f"(true {center().round(3)})", flush=True)
        _save_annotated(rgb, dets, OUT / "clear_island_detect.png", TARGET)

        # ---------- APPROACH: raise the left arm clear, keep the right tucked, DRIVE up ----------
        sim.set_head(0.0, 0.0)
        sim.set_arm_joints("left", [0.0] * 7)           # raise the left arm clear of the island
        sim.set_arm_joints("right", STOW_RIGHT)         # keep the right arm tucked down
        step(25, base_obs, frz=True)                    # let the left arm rise (still back)
        drive(base_obs, base_grasp, 48, frz=True)       # smooth wheeled drive up to the island
        print(f"[clear] after approach, mug at {center().round(3)}", flush=True)

        # ---------- GRASP at the PERCEIVED point (IK + weld, frozen reach; ex 07/08) ----------
        grasp = Pmug + WRIST_OFFSET
        ok = False
        for tgt in (grasp + np.array([0.0, 0.0, 0.15]), grasp):
            ok = sim.set_arm_pose("left", tgt, sim.top_down_quat, pos_tol=0.03, ori_tol=0.35)
            step(55, base_grasp, frz=True)
        step(30, base_grasp, grip=1.0, frz=True)
        print(f"[clear] reached grasp (ik ok={ok}); mug at {center().round(3)}", flush=True)
        if not ok:
            print("[clear] FAIL — IK did not solve at the perceived point", flush=True)
            return 1

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

        # ---------- LIFT + TRANSPORT + PLACE (weld carries the mug; no more freeze) ----------
        sim.set_arm_pose("left", grasp + np.array([0.0, 0.0, 0.22]), sim.top_down_quat,
                         pos_tol=0.04, ori_tol=0.4)
        step(60, base_grasp, grip=1.0)
        step(25, base_grasp, grip=1.0)                  # hold the mug up so the lift reads clearly
        z_lift = float(center()[2])
        body = "?"
        if mug_rb is not None:
            body = np.asarray(mug_rb.get_world_poses()[0], dtype=float).reshape(-1)[:3].round(3)
        print(f"[clear] lifted; mug bbox {center().round(3)} body {body} (rose {z_lift - z_start:+.3f})", flush=True)

        drop = Pmug + np.array([-0.09, -0.30, 0.0])          # clear spot to the side, deeper on the island
        drop[0] = min(drop[0], -0.70)                        # keep clear of the near edge (~-0.56) so it can't roll off
        for dz in (0.18, 0.04):
            okp = sim.set_arm_pose("left", drop + WRIST_OFFSET + np.array([0.0, 0.0, dz]),
                                   sim.top_down_quat, pos_tol=0.05, ori_tol=0.5)
            step(50, base_grasp, grip=1.0)
        step(15, base_grasp, grip=1.0)                       # hold still -> zero the hand velocity
        print(f"[clear] over drop (ik ok={okp}); mug at {center().round(3)}", flush=True)

        fj.CreateJointEnabledAttr().Set(False)               # release the weld
        if mug_rb is not None and rest_p is not None:        # set the mug down UPRIGHT on the surface
            rp = np.asarray(rest_p, dtype=np.float32).reshape(1, 3)
            ro = np.asarray(rest_o, dtype=np.float32).reshape(1, 4)
            placed = rp + np.array([[drop[0] - Pmug[0], drop[1] - Pmug[1], 0.0]], dtype=np.float32)
            mug_rb.set_world_poses(placed, ro)               # rest orientation = upright; rest z = on the island
            mug_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))
        step(55, base_grasp, grip=0.0)                       # open the hand; the mug rests upright

        c_end = center()
        moved = float(np.linalg.norm(c_end[:2] - Pmug[:2]))
        rose = z_lift - z_start
        ok_demo = bool(np.all(np.isfinite(c_end)) and rose > 0.07
                       and 0.12 < moved < 0.6 and 0.80 < c_end[2] < 1.05)   # rests on the island, not the floor
        print(f"[clear] mug z {z_start:.3f} ->lift {z_lift:.3f} (rose {rose:+.3f}); moved {moved:.3f} m "
              f"to {c_end.round(3)}  {'SUCCESS' if ok_demo else 'FAIL'}", flush=True)

        if GIF and frames:
            gif = OUT / "clear_island.gif"
            frames[0].save(gif, save_all=True, append_images=frames[1:], duration=70, loop=0)
            frames[0].save(OUT / "clear_island_first.png")
            frames[-1].save(OUT / "clear_island_last.png")
            if vid is not None:
                vid.save()
            print(f"[clear] wrote {gif.name} + clear_island.mp4 ({len(frames)} frames) + stills", flush=True)
        return 0 if ok_demo else 1
    finally:
        sim.close()


def _save_annotated(rgb, dets, path, target):
    try:
        import cv2
        vis = np.ascontiguousarray(rgb[:, :, ::-1]).copy()
        for name, sc, bx in dets:
            if sc < 0.12 and name != target:
                continue
            x0, y0, x1, y1 = bx.astype(int)
            col = (0, 200, 255) if name == target else (0, 255, 0)
            cv2.rectangle(vis, (x0, y0), (x1, y1), col, 2)
            cv2.putText(vis, f"{name} {sc:.2f}", (x0, max(0, y0 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        cv2.imwrite(str(path), vis)
    except Exception:  # noqa: BLE001
        from PIL import Image
        Image.fromarray(rgb).save(path)


if __name__ == "__main__":
    raise SystemExit(main())
