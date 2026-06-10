"""Gymnasium environment wrapping the R2D3 facade for RL pipelines.

Observation = head RGB + proprioception (left-arm joints, lift, wrist wrench).
Action      = end-effector position delta (+ gripper) by default, or joint deltas.
Reward / termination are user-supplied callables over the R2D3 ``Observation``,
so the same env shell serves any task.

    from isaac_sim.r2d3_sim.envs.rl_env import R2D3Env
    env = R2D3Env(control="ee_delta", reward_fn=my_reward, done_fn=my_done)
    obs, info = env.reset(); obs, r, term, trunc, info = env.step(env.action_space.sample())

Requires ``gymnasium`` (pip install gymnasium) for the spaces; the module still
imports without it (the class raises a clear error on construction).
"""
from __future__ import annotations

from typing import Callable, Optional, Sequence

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
    _BASE = gym.Env
    _HAVE_GYM = True
except Exception:  # noqa: BLE001
    gym = None
    spaces = None
    _BASE = object
    _HAVE_GYM = False

from ..r2d3 import R2D3


class R2D3Env(_BASE):
    """Single-arm R2D3 manipulation env (left arm)."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, *, end_effector: str = "dexterous", control: str = "ee_delta",
                 camera: str = "head", max_steps: int = 300, ee_step: float = 0.03,
                 joint_step: float = 0.05, settle_steps: int = 6,
                 reward_fn: Optional[Callable] = None, done_fn: Optional[Callable] = None,
                 **r2d3_kwargs):
        if not _HAVE_GYM:
            raise RuntimeError("R2D3Env needs gymnasium — `pip install gymnasium`.")
        if control not in ("ee_delta", "joint_delta", "joint_abs"):
            raise ValueError(f"unknown control mode {control!r}")
        super().__init__()
        self.control = control
        self.camera = camera
        self.max_steps = max_steps
        self.ee_step = ee_step
        self.joint_step = joint_step
        self.settle_steps = settle_steps
        self._reward_fn = reward_fn or (lambda obs: 0.0)
        self._done_fn = done_fn or (lambda obs: False)
        self._n = 0

        self.sim = R2D3(end_effector=end_effector, enable_cameras=True,
                        enable_ros=False, **r2d3_kwargs)

        h, w = self.sim.cameras._res[1], self.sim.cameras._res[0]
        self.observation_space = spaces.Dict({
            "rgb": spaces.Box(0, 255, (h, w, 3), dtype=np.uint8),
            "arm_q": spaces.Box(-np.pi, np.pi, (7,), dtype=np.float32),
            "lift": spaces.Box(0.0, 1.0, (1,), dtype=np.float32),
            "wrench": spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32),
        })
        if control == "ee_delta":
            # dx, dy, dz, gripper
            self.action_space = spaces.Box(-1.0, 1.0, (4,), dtype=np.float32)
        else:
            # 7 joint deltas/targets + gripper
            self.action_space = spaces.Box(-1.0, 1.0, (8,), dtype=np.float32)

    # ----- gym API -----
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.sim.reset()
        self._n = 0
        return self._obs(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        self._apply(action)
        self.sim.step(n=self.settle_steps)
        self._n += 1
        obs = self._obs()
        r2d3_obs = self.sim.get_observation()
        reward = float(self._reward_fn(r2d3_obs))
        terminated = bool(self._done_fn(r2d3_obs))
        truncated = self._n >= self.max_steps
        return obs, reward, terminated, truncated, {}

    def render(self):
        return self.sim.get_image(self.camera)

    def close(self):
        self.sim.close()

    # ----- helpers -----
    def _apply(self, action: np.ndarray) -> None:
        if self.control == "ee_delta":
            pos, _ = self.sim.get_ee_pose("left")
            target = pos + action[:3] * self.ee_step
            self.sim.set_arm_pose("left", target, self.sim.top_down_quat)
            self.sim.set_gripper("left", float((action[3] + 1.0) / 2.0))
        else:
            q = np.array([self.sim.get_joint_state().get(f"l_joint{i}") for i in range(1, 8)])
            if self.control == "joint_delta":
                q = q + action[:7] * self.joint_step
            else:  # joint_abs in [-pi, pi]
                q = action[:7] * np.pi
            self.sim.set_arm_joints("left", q.tolist())
            self.sim.set_gripper("left", float((action[7] + 1.0) / 2.0))

    def _obs(self) -> dict:
        js = self.sim.get_joint_state()
        return {
            "rgb": self.sim.get_image(self.camera),
            "arm_q": np.array([js.get(f"l_joint{i}") for i in range(1, 8)], dtype=np.float32),
            "lift": np.array([self.sim.get_lift()], dtype=np.float32),
            "wrench": np.asarray(self.sim.get_wrench("left"), dtype=np.float32),
        }
