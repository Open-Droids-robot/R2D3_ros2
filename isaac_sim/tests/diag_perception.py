"""De-risk probe for the ML-driven kitchen demo: does the open-vocabulary detector
fire on the simulated YCB objects, and is the pixel->world unprojection accurate?

    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_perception.py [--back 0.3 --tilt -0.62]

Loads the kitchen, stows the arm + points the head camera down at the island, runs
OWL-ViT (via r2d3_sim.perception, in a subprocess) on the head-cam RGB, unprojects
the detected 'mug' to a world point and compares it to the mug's true pose, and
saves an annotated frame to isaac_sim/tests/captures/perception_head.png.

Finding: a COCO detector (torchvision Faster R-CNN) does NOT recognise the sim YCB
props (a red mug -> 'chair'); OWL-ViT (open-vocab, CLIP-based) does, from a text query.
"""
import numpy as np
from pathlib import Path

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", type=float, default=-0.45)
    ap.add_argument("--back", type=float, default=0.0)   # move base +x (back from island) for observe
    ap.add_argument("--lift", type=float, default=None)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    from isaac_sim.r2d3_sim import scenes
    man = {}
    sim = R2D3(end_effector="dexterous", mobile=True, headless=True, enable_cameras=True,
               camera_resolution=(1280, 960),
               setup=lambda w: man.update(scenes.load("kitchen", w)))
    try:
        import omni.usd
        from pxr import UsdGeom, Gf
        from isaac_sim.r2d3_sim import scene as scene_mod
        stage = omni.usd.get_context().get_stage()

        sim.reset()
        sim.go_home()
        sim.set_lift(float(args.lift if args.lift is not None else man.get("lift", 0.90)))
        sx, sy, yaw = man["spawn"]
        q = h.yaw_quat(yaw)
        base_pos = np.array([sx + args.back, sy, 0.27])  # +back = farther from the island (faces -x)
        for _ in range(60):                            # place + hold + settle objects
            sim.set_base_pose(base_pos, q)
            sim.world.step(render=False)

        # Stow the LEFT arm out of the (left-offset, downward) head-cam cone: FK-search
        # joint space for a hand pose that is BEHIND the camera (x >= cam_x) and LOW.
        from isaac_sim.r2d3_sim import sim_topics as st
        sim.ik.sync_base()
        cam_x = -0.27
        best = None
        for j1 in np.linspace(-1.6, 1.6, 9):
            for j2 in np.linspace(-2.2, 2.2, 15):
                for j4 in np.linspace(-2.2, 0.0, 6):
                    cand = [float(j1), float(j2), 0.0, float(j4), 0.0, 0.0, 0.0]
                    try:
                        pos, _ = sim.ik.fk(cand)
                    except Exception:  # noqa: BLE001
                        continue
                    if pos[0] >= cam_x - 0.05 and 0.25 < pos[2] < 1.35:   # behind cam + low
                        score = pos[2] - 0.4 * (pos[0] - cam_x)            # prefer low + behind
                        if best is None or score < best[0]:
                            best = (score, cand, pos)
        if best is not None:
            ql = best[1]
            sim.set_arm_joints("left", ql)
            print(f"[perc] stow q_left={[round(v,2) for v in ql]} hand={np.asarray(best[2]).round(2)}", flush=True)
        else:
            ql = None
            print("[perc] stow: no behind+low FK pose found", flush=True)
        for _ in range(25):
            sim.set_base_pose(base_pos, q)
            sim.world.step(render=False)

        sim.set_head(0.0, args.tilt)
        h.set_lighting(dome=400.0, key=4000.0, fill=2000.0)
        for _ in range(20):                            # settle physics
            sim.set_base_pose(base_pos, q)
            sim.world.step(render=False)
        for _ in range(15):                            # REFRESH render products at the new pose
            sim.set_base_pose(base_pos, q)
            sim.world.step(render=True)

        # --- head-cam geometry: where is it, which way does it look? ---
        cam_path0 = sim.cameras._prims["head"]
        M0 = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(cam_path0))
        cpos0 = M0.ExtractTranslation()
        fwd0 = Gf.Vec3d(0, 0, -1) * M0.ExtractRotationMatrix()   # USD cam looks down local -Z
        print(f"[perc] tilt={args.tilt} back={args.back} cam_pos={np.array(cpos0).round(2)} "
              f"fwd={np.array(fwd0).round(2)}", flush=True)
        lh = scene_mod.world_range(h.prim_path("l_hand_link"))
        print(f"[perc] l_hand x[{lh[0][0]:.2f},{lh[1][0]:.2f}] z[{lh[0][2]:.2f},{lh[1][2]:.2f}]", flush=True)

        rgb, depth = sim.get_image("head", depth=True)
        H, W = depth.shape
        # depth grid: rows top->bottom, cols left->right (what's in front of each region)
        print("[perc] depth grid (m):", flush=True)
        for fy in (0.2, 0.5, 0.8):
            row = [depth[int(fy * H), int(fx * W)] for fx in (0.2, 0.5, 0.8)]
            print("        " + "  ".join(f"{v:5.2f}" for v in row), flush=True)
        dmed = float(np.median(depth[depth > 0])) if (depth > 0).any() else -1
        print(f"[perc] head rgb={rgb.shape} depth={depth.shape} median_depth={dmed:.2f} m "
              f"rgb_mean={rgb.mean():.0f}", flush=True)

        # --- intrinsics + world pose from the head Camera prim ---
        cam_path = sim.cameras._prims["head"]
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam = UsdGeom.Camera(cam_prim)
        fl = cam.GetFocalLengthAttr().Get()
        fx = fl / cam.GetHorizontalApertureAttr().Get() * W
        fy = fl / cam.GetVerticalApertureAttr().Get() * H
        cx, cy = W / 2.0, H / 2.0
        M = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
        cam_pos = np.array(M.ExtractTranslation())
        R = M.ExtractRotationMatrix()

        def pixel_to_world(u, v):
            d = float(depth[int(v), int(u)])
            if d <= 0:
                return None
            ray = Gf.Vec3d((u - cx) / fx, -(v - cy) / fy, -1.0).GetNormalized()  # USD cam: -Z fwd
            wd = ray * R                                   # rotate to world (row-vec * R)
            return cam_pos + d * np.array([wd[0], wd[1], wd[2]])

        # --- OWL-ViT open-vocabulary detector (runs in an isolated subprocess) ---
        from isaac_sim.r2d3_sim import perception
        QUERIES = ["a red mug", "a bowl", "a cracker box", "a soup can", "a mustard bottle"]
        dets = perception.detect(rgb, QUERIES, threshold=0.05)
        print(f"[perc] OWL-ViT detections (score>0.05):", flush=True)
        for name, sc, bx in dets[:10]:
            print(f"        {name:18s} {sc:.2f}  box={bx.round(0)}", flush=True)

        # --- accuracy: unproject the best 'mug' box vs the mug's true centre ---
        mug = next((d for d in dets if "mug" in d[0]), None)
        if mug is not None:
            x0, y0, x1, y1 = mug[2]
            u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            pw = pixel_to_world(u, v)
            lo, hi = scene_mod.world_range(man["objects"]["mug"])
            true_c = (np.asarray(lo) + np.asarray(hi)) / 2.0
            if pw is not None:
                err = float(np.linalg.norm(pw[:2] - true_c[:2]))
                print(f"[perc] '{mug[0]}' {mug[1]:.2f} pixel({u:.0f},{v:.0f}) -> world {pw.round(3)} | "
                      f"true mug {true_c.round(3)} | xy-error {err:.3f} m", flush=True)
                print(f"[perc] RESULT: {'PASS' if err < 0.12 else 'FAIL'} "
                      f"(mug detected + unprojection {'accurate' if err < 0.12 else 'off'})", flush=True)
            else:
                print("[perc] RESULT: FAIL — no depth at mug pixel", flush=True)
        else:
            print(f"[perc] RESULT: FAIL — OWL-ViT found no mug. Got: {[d[0] for d in dets[:5]]}", flush=True)

        # --- annotated frame ---
        try:
            import cv2
            vis = np.ascontiguousarray(rgb[:, :, ::-1]).copy()   # RGB->BGR for cv2
            for name, sc, bx in dets:
                x0, y0, x1, y1 = bx.astype(int)
                cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(vis, f"{name} {sc:.2f}", (x0, max(0, y0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imwrite(str(OUT / "perception_head.png"), vis)
            print(f"[perc] wrote {OUT / 'perception_head.png'}", flush=True)
        except Exception as e:  # noqa: BLE001
            from PIL import Image
            Image.fromarray(rgb).save(OUT / "perception_head.png")
            print(f"[perc] wrote raw frame (cv2 draw failed: {e})", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
