"""Perception → action loop on top of the R2D3 facade.

The in-process analogue of a VLM / vision policy: grab a camera frame + the
observation, call an external model, decode a high-level action, apply it, step.
The ``policy`` callable receives ``(rgb, Observation)`` and returns an action
dict; supported keys map onto the facade setters.
"""
from __future__ import annotations

from typing import Callable


def apply_action(sim, action: dict) -> None:
    """Apply a high-level action dict to the sim.

    Recognised keys:
      arm_pose   = {"side","position","quat"(optional)}  -> set_arm_pose (IK)
      arm_joints = {"side","q"}                          -> set_arm_joints
      gripper    = {"side","frac"}                       -> set_gripper
      head       = (pan, tilt)                            -> set_head
      lift       = float                                  -> set_lift
      base_pose  = (position, quat_wxyz)                  -> set_base_pose
    """
    if "arm_pose" in action:
        a = action["arm_pose"]
        sim.set_arm_pose(a["side"], a["position"], a.get("quat"))
    if "arm_joints" in action:
        a = action["arm_joints"]
        sim.set_arm_joints(a["side"], a["q"])
    if "gripper" in action:
        a = action["gripper"]
        sim.set_gripper(a["side"], a["frac"])
    if "head" in action:
        sim.set_head(*action["head"])
    if "lift" in action:
        sim.set_lift(action["lift"])
    if "base_pose" in action:
        sim.set_base_pose(*action["base_pose"])


class PerceptionLoop:
    """Drive the robot from a perception model: obs -> action -> step."""

    def __init__(self, sim, policy: Callable[["object", "object"], dict],
                 *, camera: str = "head"):
        self.sim = sim
        self.policy = policy
        self.camera = camera

    def run(self, n_iters: int, *, settle_steps: int = 30) -> list:
        """Run ``n_iters`` perception-action cycles. Returns the action history."""
        history = []
        for _ in range(n_iters):
            rgb = self.sim.get_image(self.camera)
            obs = self.sim.get_observation()
            action = self.policy(rgb, obs)
            history.append(action)
            if action:
                apply_action(self.sim, action)
            self.sim.step(n=settle_steps)
        return history
