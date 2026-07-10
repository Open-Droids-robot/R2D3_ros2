"""Polished full-body motion reel — a single smooth sequence that exercises
every actuated subsystem so a viewer can see the whole robot works.

This supersedes the old ``diag_motion_gif.py``: that script *snapped* each joint
target and let the PhysX drive lunge to it (jerky, "weird" motion). Here every
channel (both 7-DOF arms, head pan/tilt, body lift, hands, left-arm Cartesian
IK) is driven along an **eased** trajectory — the commanded target itself is
interpolated with a smoothstep curve each frame, so the drive follows a clean
accel/decel profile. The robot stands on a visible studio floor with its wheels
grounded (it is NOT floating); gravity stays off on the articulation (the real
RM75-B does gravity-comp — see robot.py), exactly as in the SDK.

Output (isaac_sim/tests/captures/):
  * motion.mp4              720p / 30fps H.264 reel  (embed in the deck)
  * motion_<phase>.png      stills for quick eyeballing / slides

    scripts/isaacsim_ros2.sh isaac_sim/tests/motion_reel.py [--ee dexterous|gripper]
"""
import argparse
from pathlib import Path

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h
from isaac_sim.r2d3_sim.robot import (
    HOME_LEFT_ARM, HOME_RIGHT_ARM, HOME_LIFT_M,
    HOME_HEAD_PAN_RAD, HOME_HEAD_TILT_RAD,
)

OUT = Path(__file__).resolve().parents[1] / "tests" / "captures"
SIZE = (1280, 720)


def smoothstep(a: float) -> float:
    """Ease-in/ease-out on a normalized [0,1] parameter (zero end-velocities)."""
    a = max(0.0, min(1.0, a))
    return a * a * (3.0 - 2.0 * a)


def main() -> int:
    ap = argparse.ArgumentParser(description="Render the full-body motion reel")
    ap.add_argument("--ee", choices=["dexterous", "gripper"], default="dexterous")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    sim = R2D3(end_effector=args.ee, headless=True, enable_cameras=False)
    try:
        import omni.replicator.core as rep
        from isaac_sim.r2d3_sim import scene as scene_mod

        sim.reset()
        sim.go_home()
        for _ in range(40):                      # settle physics on the home pose
            sim.world.step(render=False)

        # ---- Studio: frame the whole robot from its world bounding box -------
        rmin, rmax = scene_mod.world_range(sim._robot_prim)
        rmin = np.asarray(rmin, float); rmax = np.asarray(rmax, float)
        ctr = (rmin + rmax) / 2.0
        floor_z = float(rmin[2])                 # robot's lowest point = wheel contact

        # Coloured studio so the white robot reads cleanly; a big floor + back/side
        # walls give the eye a clear ground plane (robot is visibly grounded).
        h.set_lighting(dome=130.0, key=3600.0, fill=2500.0)
        scene_mod.add_visual_box("/Preview/floor",
                                 (float(ctr[0]), float(ctr[1]), floor_z - 0.01),
                                 (16.0, 16.0, 0.02), (0.22, 0.36, 0.50))
        scene_mod.add_visual_box("/Preview/wall_back",
                                 (float(ctr[0]) - 2.6, float(ctr[1]), floor_z + 1.4),
                                 (0.1, 16.0, 5.0), (0.13, 0.16, 0.24))
        scene_mod.add_visual_box("/Preview/wall_side",
                                 (float(ctr[0]), float(ctr[1]) + 3.2, floor_z + 1.4),
                                 (16.0, 0.1, 5.0), (0.17, 0.13, 0.22))

        # Front-right 3/4 view (robot faces +X). Framed to fit the WHOLE robot
        # at its tallest moments — lift fully up (~0.92 m) and arms raised above
        # the head — while still showing the wheels + floor contact at the
        # bottom. Aim at mid-height (~1 m) and stand back ~4.7 m so nothing clips.
        cx, cy = float(ctr[0]) - 0.10, float(ctr[1])
        eye = np.array([cx + 3.7, cy - 3.0, floor_z + 1.15])
        look = np.array([cx, cy, floor_z + 1.00])
        cam = rep.functional.create.camera(
            position=tuple(float(v) for v in eye),
            look_at=tuple(float(v) for v in look))
        rp = rep.create.render_product(str(cam.GetPath()), SIZE)
        ann = rep.AnnotatorRegistry.get_annotator("rgb"); ann.attach(rp)

        vid = h.Mp4Writer(OUT / "motion.mp4", size=SIZE, fps=30)
        last = {"rgb": None}

        def grab():
            a = np.asarray(ann.get_data(do_array_copy=True))
            rgb = h.rgba_to_rgb(a)
            last["rgb"] = rgb
            vid.add(rgb)

        def snap(name):
            if last["rgb"] is not None:
                from PIL import Image
                Image.fromarray(last["rgb"]).save(OUT / f"motion_{name}.png")

        # ---- Channel state (interpolate the *target* of each subsystem) ------
        state = {
            "L": np.array(HOME_LEFT_ARM, float),
            "R": np.array(HOME_RIGHT_ARM, float),
            "lift": float(HOME_LIFT_M),
            "head": np.array([HOME_HEAD_PAN_RAD, HOME_HEAD_TILT_RAD], float),
            "hand": 0.0,                          # 0 = open, 1 = closed (both hands)
        }

        def apply():
            sim.set_arm_joints("left", list(state["L"]))
            sim.set_arm_joints("right", list(state["R"]))
            sim.set_lift(float(state["lift"]))
            sim.set_head(float(state["head"][0]), float(state["head"][1]))
            sim.set_gripper("left", float(state["hand"]))
            sim.set_gripper("right", float(state["hand"]))

        def transition(targets, steps=40, hold=6):
            """Ease every named channel from its current value to the target,
            stepping + grabbing one frame per step, then hold a beat."""
            start = {k: (v.copy() if isinstance(v, np.ndarray) else v)
                     for k, v in state.items()}
            for i in range(steps):
                a = smoothstep((i + 1) / steps)
                for k, tgt in targets.items():
                    if isinstance(state[k], np.ndarray):
                        state[k] = start[k] + (np.asarray(tgt, float) - start[k]) * a
                    else:
                        state[k] = start[k] + (float(tgt) - start[k]) * a
                apply(); sim.world.step(render=True); grab()
            for _ in range(hold):
                apply(); sim.world.step(render=True); grab()

        # ---- RTX warm-up (no capture) ----------------------------------------
        for _ in range(16):
            apply(); sim.world.step(render=True)

        # ---- Reel ------------------------------------------------------------
        transition({}, steps=1, hold=20)                 # hold home; establish shot
        snap("home")

        # ARMS: symmetric sweep up, across, back home (vetted poses, now eased)
        transition({"L": [0.6, -0.7, 0.6, -1.1, 0.6, 0.7, 0.5],
                    "R": [0.6, -0.7, 0.6, -1.1, 0.6, 0.7, 0.5]}, steps=44, hold=8)
        snap("arms")
        transition({"L": [-0.5, 0.3, -0.5, -0.5, -0.4, -0.3, -0.3],
                    "R": [-0.5, 0.3, -0.5, -0.5, -0.4, -0.3, -0.3]}, steps=44, hold=8)
        transition({"L": HOME_LEFT_ARM, "R": HOME_RIGHT_ARM}, steps=40, hold=10)

        # HEAD: pan/tilt look-around, back to neutral
        transition({"head": [0.6, -0.2]}, steps=24, hold=4)
        transition({"head": [-0.6, 0.2]}, steps=32, hold=4)
        transition({"head": [0.0, HOME_HEAD_TILT_RAD]}, steps=24, hold=8)
        snap("head")

        # LIFT: torso up, down, back to home height
        transition({"lift": 0.92}, steps=42, hold=8)
        snap("lift")
        transition({"lift": 0.30}, steps=42, hold=8)
        transition({"lift": HOME_LIFT_M}, steps=36, hold=10)

        # HANDS: close then open (both)
        transition({"hand": 1.0}, steps=28, hold=10)
        snap("hands")
        transition({"hand": 0.0}, steps=28, hold=8)

        # IK: smooth left-arm Cartesian reach out + back (top-down wrist)
        ee0, _ = sim.get_ee_pose("left")
        reach = ee0 + np.array([0.12, 0.0, -0.16])
        for i in range(34):
            a = smoothstep((i + 1) / 34)
            sim.set_arm_pose("left", ee0 + (reach - ee0) * a, sim.top_down_quat,
                             pos_tol=0.03, ori_tol=0.35)
            sim.world.step(render=True); grab()
        for _ in range(8):
            sim.world.step(render=True); grab()
        snap("reach")
        for i in range(34):
            a = smoothstep((i + 1) / 34)
            sim.set_arm_pose("left", reach + (ee0 - reach) * a, sim.top_down_quat,
                             pos_tol=0.03, ori_tol=0.35)
            sim.world.step(render=True); grab()
        for _ in range(16):
            sim.world.step(render=True); grab()

        ann.detach(); rp.destroy()
        means = "n/a"
        if len(vid):
            r = last["rgb"]
            means = f"{float(np.asarray(r).mean()):.0f}" if r is not None else "n/a"
        vid.save()
        print(f"[reel] wrote motion.mp4  frames={len(vid)}  ee={args.ee}  "
              f"last-frame brightness={means}", flush=True)
        print("[reel] DONE", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
