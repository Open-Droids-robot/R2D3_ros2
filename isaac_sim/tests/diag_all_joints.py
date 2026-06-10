"""Comprehensive motion check — command every actuated part and confirm it moves.

    scripts/isaacsim_ros2.sh isaac_sim/tests/diag_all_joints.py [--ee dexterous|gripper] [--mobile]

Fixed-base run exercises: both 7-DOF arms, head pan/tilt, body lift, both hands
(close/open), and left-arm IK. The --mobile run exercises the base (translate +
rotate) and the AGV wheels (drive + casters). Prints PASS/FAIL per joint with the
measured delta; exits non-zero if any expected motion failed.
"""
import argparse

import numpy as np

from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim import helpers as h
# NOTE: sim_topics is imported INSIDE main(), AFTER R2D3() — it reads R2D3_EE at
# import time, so importing it early would lock the end-effector to the default.

REV = 0.02   # rad — minimum revolute motion to count as "moved"
LIN = 0.02   # m   — minimum prismatic motion
_FAIL = []


def chk(label, before, after, thresh, unit=" rad"):
    d = after - before
    ok = abs(d) >= thresh
    print(f"  [{'PASS' if ok else 'FAIL'}] {label:28s} delta={d:+.4f}{unit}  ({before:.3f} -> {after:.3f})", flush=True)
    if not ok:
        _FAIL.append(label)
    return ok


def grp(title):
    print(f"\n=== {title} ===", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="R2D3 full-body motion check")
    ap.add_argument("--ee", choices=["dexterous", "gripper"], default="dexterous")
    ap.add_argument("--mobile", action="store_true", help="test the base + wheels instead of the upper body")
    args = ap.parse_args()

    sim = R2D3(end_effector=args.ee, mobile=args.mobile, headless=True, enable_cameras=False)
    try:
        from isaac_sim.r2d3_sim import sim_topics as t   # now reflects the real EE
        sim.reset()
        jp = sim.get_joint_positions  # callable -> {name: pos}
        print(f"[diag] ee={args.ee} mobile={args.mobile}  num_dof={sim.robot.num_dof}", flush=True)

        if not args.mobile:
            # ---- ARMS (7 DOF each) ----
            grp("Arms — 7 DOF each")
            for side in ("left", "right"):
                p = side[0]
                pre = jp()
                sim.set_arm_joints(side, [0.4, -0.5, 0.4, -0.8, 0.4, 0.5, 0.3])
                sim.step(render=False, n=120)
                post = jp()
                for i in range(1, 8):
                    chk(f"{side} {p}_joint{i}", pre[f"{p}_joint{i}"], post[f"{p}_joint{i}"], REV)
                sim.set_arm_joints(side, [0.0] * 7)
                sim.step(render=False, n=60)

            # ---- HEAD ----
            grp("Head — pan + tilt")
            pre = jp()
            sim.set_head(0.4, -0.5)
            sim.step(render=False, n=80)
            post = jp()
            chk("head_joint1 (pan)", pre["head_joint1"], post["head_joint1"], REV)
            chk("head_joint2 (tilt)", pre["head_joint2"], post["head_joint2"], REV)

            # ---- LIFT ----
            grp("Lift — platform_joint (prismatic)")
            pre = jp()
            lift0 = sim.get_lift()
            sim.set_lift(min(0.95, lift0 + 0.3))
            sim.step(render=False, n=150)
            chk("platform_joint", pre[t.LIFT_JOINT], jp()[t.LIFT_JOINT], LIN, " m")

            # ---- HANDS ----
            grp(f"Hands ({args.ee}) — close then open")
            for side in ("left", "right"):
                names = t.LEFT_HAND_JOINTS if side == "left" else t.RIGHT_HAND_JOINTS
                pre = jp()
                sim.set_gripper(side, 1.0)
                sim.step(render=False, n=90)
                post = jp()
                moved = [n for n in names if abs(post[n] - pre[n]) >= (0.004 if "finger" in n else REV)]
                need = 1 if args.ee == "gripper" else 4
                ok = len(moved) >= need
                print(f"  [{'PASS' if ok else 'FAIL'}] {side} hand close: {len(moved)}/{len(names)} joints moved "
                      f"(need >= {need})", flush=True)
                if not ok:
                    _FAIL.append(f"{side} hand")
                sim.set_gripper(side, 0.0)
                sim.step(render=False, n=60)

            # ---- IK (left arm, Cartesian) ----
            grp("IK — left EE Cartesian move")
            sim.go_home()
            sim.step(render=False, n=60)
            ee0, _ = sim.get_ee_pose("left")
            ok_ik = sim.set_arm_pose("left", ee0 + np.array([0.0, 0.0, -0.12]), sim.top_down_quat)
            sim.step(render=False, n=120)
            ee1, _ = sim.get_ee_pose("left")
            moved = float(np.linalg.norm(ee1 - ee0))
            ok = moved >= 0.05
            print(f"  [{'PASS' if ok else 'FAIL'}] left EE moved {moved:.3f} m (ik_solved={ok_ik})  "
                  f"{ee0.round(3)} -> {ee1.round(3)}", flush=True)
            if not ok:
                _FAIL.append("IK EE move")

        else:
            # ---- MOBILE BASE + WHEELS ----
            grp("Mobile base — translate +Y, wheels rolling")
            p0, q0 = sim.robot.get_base_pose()
            pre = jp()
            for i in range(40):
                d = 1.0 * (i + 1) / 40
                sim.set_base_pose(p0 + np.array([0.0, d, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
                ang = d / 0.05
                sim.set_joint_targets({
                    "joint_left_wheel": ang, "joint_right_wheel": -ang,
                    **{f"joint_swivel_wheel_{k}_2": ang for k in (1, 2, 3, 4)},
                })
                sim.step(render=False)
            p1, _ = sim.robot.get_base_pose()
            post = jp()
            chk("base translate (+Y)", p0[1], p1[1], 0.1, " m")
            for w in ("joint_left_wheel", "joint_right_wheel"):
                chk(f"drive {w}", pre[w], post[w], 0.5)
            for k in (1, 2, 3, 4):
                w = f"joint_swivel_wheel_{k}_2"
                chk(f"caster {w}", pre[w], post[w], 0.5)

            grp("Mobile base — rotate 90 deg about Z")
            pm, _ = sim.robot.get_base_pose()
            for i in range(30):
                sim.set_base_pose(pm, h.yaw_quat(90.0 * (i + 1) / 30))
                sim.step(render=False)
            _, q2 = sim.robot.get_base_pose()
            dq = float(np.linalg.norm(np.asarray(q2) - np.asarray(q0)))
            ok = dq >= 0.3
            print(f"  [{'PASS' if ok else 'FAIL'}] base rotate (quat delta={dq:.3f})  "
                  f"{np.asarray(q0).round(3)} -> {np.asarray(q2).round(3)}", flush=True)
            if not ok:
                _FAIL.append("base rotate")

        print("\n" + "=" * 52, flush=True)
        if _FAIL:
            print(f"[diag] RESULT: {len(_FAIL)} FAIL -> {_FAIL}", flush=True)
            return 1
        print("[diag] RESULT: ALL PASS — every commanded part moved", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
