"""Perception → action loop — the shape of a VLM / vision-policy controller.

    scripts/isaacsim_ros2.sh isaac_sim/examples/05_vlm_loop.py

`policy(rgb, obs)` is where a real model goes (caption the head image, predict a
grasp, emit a high-level action). Here it's a stub that nudges the arm toward a
fixed point so the loop is runnable offline.
"""
import numpy as np
from isaac_sim.r2d3_sim import R2D3
from isaac_sim.r2d3_sim.envs.vlm_loop import PerceptionLoop


def stub_policy(rgb: np.ndarray, obs) -> dict:
    """Stand-in for a VLM. Receives the head RGB (HxWx3) + the full Observation,
    returns a high-level action dict. Here: step the left EE toward a target."""
    target = np.array([0.46, -0.21, 0.52])
    cur = obs.ee_pose_left[0]
    nxt = cur + 0.4 * (target - cur)         # move 40% of the way each step
    return {"arm_pose": {"side": "left", "position": nxt.tolist()},
            "gripper": {"side": "left", "frac": 0.0}}


def main() -> int:
    with R2D3(end_effector="dexterous", headless=True, enable_cameras=True) as sim:
        sim.reset()
        sim.set_head(0.0, -0.3)
        loop = PerceptionLoop(sim, stub_policy, camera="head")
        loop.run(n_iters=5, settle_steps=30)
        print(f"[vlm] final left EE: {sim.get_ee_pose('left')[0].round(3)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
