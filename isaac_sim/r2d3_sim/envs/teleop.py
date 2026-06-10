"""Teleoperation server on top of the R2D3 facade.

Stream target poses into the sim from any source (keyboard, gamepad, VR, a
network socket, a leader arm). Targets are latched; each ``tick`` pushes the
latest targets into the robot and steps physics. ``use_ros=True`` instead reuses
the existing ROS command path (``/r2d3/sim/cmd/*`` via sim_adapter).
"""
from __future__ import annotations

from typing import Optional, Sequence


class TeleopServer:
    """Latch the latest control targets and apply them each sim tick."""

    def __init__(self, sim, *, use_ros: bool = False):
        self.sim = sim
        self.use_ros = use_ros
        if use_ros and sim._ros is None:
            sim._start_ros()
        self._latest = {}

    def submit(self, *, left_arm: Optional[Sequence[float]] = None,
               right_arm: Optional[Sequence[float]] = None,
               head: Optional[tuple] = None, lift: Optional[float] = None,
               gripper_left: Optional[float] = None, gripper_right: Optional[float] = None,
               left_ee: Optional[tuple] = None) -> None:
        """Set the latest targets (any subset). Joint targets are 7-vecs (rad);
        ``head`` is (pan, tilt); grippers are 0=open..1=closed; ``left_ee`` is
        (position, quat_wxyz) solved via IK on tick."""
        if left_arm is not None: self._latest["left_arm"] = list(left_arm)
        if right_arm is not None: self._latest["right_arm"] = list(right_arm)
        if head is not None: self._latest["head"] = tuple(head)
        if lift is not None: self._latest["lift"] = float(lift)
        if gripper_left is not None: self._latest["gripper_left"] = float(gripper_left)
        if gripper_right is not None: self._latest["gripper_right"] = float(gripper_right)
        if left_ee is not None: self._latest["left_ee"] = left_ee

    def tick(self, *, render: bool = True):
        """Apply the latest targets and advance one physics step. Returns obs."""
        l = self._latest
        if "left_ee" in l:
            pos, quat = l["left_ee"]
            self.sim.set_arm_pose("left", pos, quat)
        elif "left_arm" in l:
            self.sim.set_arm_joints("left", l["left_arm"])
        if "right_arm" in l:
            self.sim.set_arm_joints("right", l["right_arm"])
        if "head" in l:
            self.sim.set_head(*l["head"])
        if "lift" in l:
            self.sim.set_lift(l["lift"])
        if "gripper_left" in l:
            self.sim.set_gripper("left", l["gripper_left"])
        if "gripper_right" in l:
            self.sim.set_gripper("right", l["gripper_right"])
        return self.sim.step(render=render)

    def spin(self, n_ticks: int, *, render: bool = True):
        """Convenience loop: tick ``n_ticks`` times (targets are re-applied each
        tick, so external code can keep calling ``submit`` between/within)."""
        for _ in range(n_ticks):
            self.tick(render=render)
