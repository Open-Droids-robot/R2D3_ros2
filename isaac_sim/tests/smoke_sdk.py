"""Smoke-test the R2D3 SDK facade end-to-end (headless).

    scripts/isaacsim_ros2.sh isaac_sim/tests/smoke_sdk.py
"""
import sys

import numpy as np
from isaac_sim.r2d3_sim import R2D3


def main() -> int:
    sim = R2D3(end_effector="dexterous", headless=True)
    try:
        sim.reset()
        print(f"[smoke] reset OK; num_dof={sim.robot.num_dof}", flush=True)

        ok = sim.set_arm_pose("left", [0.45, -0.21, 0.55], sim.top_down_quat)
        print(f"[smoke] set_arm_pose(left) ok={ok}", flush=True)
        sim.set_gripper("left", 1.0)       # close
        sim.set_head(0.0, -0.4)
        sim.set_lift(0.55)
        sim.step(n=40)

        for cam in sim.cameras.names:
            img = sim.get_image(cam)
            print(f"[smoke] camera {cam}: shape={img.shape} mean={float(np.mean(img)):.1f}", flush=True)

        w = sim.get_wrench("left")
        pos, quat = sim.get_ee_pose("left")
        js = sim.get_joint_state()
        obs = sim.get_observation()
        print(f"[smoke] wrench_left={np.asarray(w).round(2)}", flush=True)
        print(f"[smoke] ee_left pos={pos.round(3)} quat={quat.round(3)}", flush=True)
        print(f"[smoke] joints={len(js.names)}  lift={sim.get_lift():.3f}", flush=True)
        print(f"[smoke] observation images={list(obs.images.keys())}", flush=True)
        print("[smoke] DONE", flush=True)
        return 0
    finally:
        sim.close()


if __name__ == "__main__":
    sys.exit(main())
