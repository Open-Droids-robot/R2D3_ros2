"""RL demo — a short random rollout in the Gymnasium R2D3Env.

    pip install gymnasium      # into the isaac env, once
    scripts/isaacsim_ros2.sh isaac_sim/examples/04_rl_env.py

Plug your own reward_fn / done_fn (callables over the R2D3 Observation) and hand
the env to any RL library (SB3, CleanRL, rllib, ...).
"""
import argparse

import numpy as np


def reach_reward(obs):
    """Toy reward: negative distance of the left EE to a target point."""
    target = np.array([0.5, -0.2, 0.5])
    return -float(np.linalg.norm(obs.ee_pose_left[0] - target))


def main() -> int:
    ap = argparse.ArgumentParser(description="RL rollout (switchable end-effector)")
    ap.add_argument("--ee", choices=["dexterous", "gripper"], default="dexterous")
    ee = ap.parse_args().ee
    from isaac_sim.r2d3_sim.envs.rl_env import R2D3Env
    env = R2D3Env(control="ee_delta", max_steps=20, reward_fn=reach_reward, end_effector=ee)
    obs, _ = env.reset()
    print(f"[rl] obs keys={list(obs.keys())}  rgb={obs['rgb'].shape}  action={env.action_space.shape}")
    total = 0.0
    for t in range(20):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)
        total += reward
        if terminated or truncated:
            print(f"[rl] episode ended at t={t} (term={terminated} trunc={truncated})")
            break
    print(f"[rl] random rollout return={total:.3f}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
