"""Policy base class for R2D3 V1 participants.

Participants implement a subclass of :class:`Policy` and point the
``policy`` parameter of the :class:`R2D3Model` lifecycle node at their
module. The host node loads the class dynamically at configure-time.

Example
-------

.. code-block:: python

    from r2d3_model.policy import Policy
    from r2d3_model_interfaces.msg import Observation

    class MyPolicy(Policy):
        def on_configure(self) -> None:
            # Heavy one-time setup (model loading, etc.)
            self.checkpoint = load_checkpoint("/workspace/policy.pt")

        def on_activate(self) -> None:
            # Lightweight reset between trials
            pass

        def on_task(self, task_id: str, task_seed: int, trial_index: int) -> None:
            # Called once at the start of each trial. Stash any task-specific
            # state here. Do NOT block here — the host runs `step` in a loop.
            self.target_zone = self._lookup_zone(task_id)

        def step(self, obs: Observation):
            # Called at the rate observations arrive. Return one of:
            #   None                     no command this tick
            #   ('movej', joint_targets)
            #   ('movej_p', pose)
            #   ('gripper', side, 0..1000)
            #   ('lift', height_mm, speed_pct)
            ...

        def on_deactivate(self) -> None:
            pass

        def on_cleanup(self) -> None:
            pass
"""

from __future__ import annotations

from typing import Any, Optional


class Policy:
    """Base class for participant policies.

    The host lifecycle node owns the ROS 2 graph; the policy only sees
    :class:`Observation` messages and returns command tuples. This keeps
    the policy framework-agnostic (no rclpy imports required).
    """

    def __init__(self) -> None:
        pass

    # --- Lifecycle hooks (override as needed) ------------------------------

    def on_configure(self) -> None:
        """Called once when the lifecycle node transitions to *inactive*.

        Use for heavy one-time setup: load checkpoints, allocate buffers,
        etc. The host blocks on this; long-running setup is fine.
        """

    def on_activate(self) -> None:
        """Called when the lifecycle node transitions to *active*.

        Typically a no-op; reset trial-local state in :meth:`on_task` instead.
        """

    def on_task(self, task_id: str, task_seed: int, trial_index: int) -> None:
        """Called once per trial, just before :meth:`step` starts.

        Parameters
        ----------
        task_id : str
            Slug of the task being run (e.g. ``"pick_and_place"``).
        task_seed : int
            Deterministic seed shared with the engine's scene spawn.
        trial_index : int
            0..N index of this trial within a multi-trial run.
        """

    def step(self, obs: Any) -> Optional[tuple]:
        """Called at observation rate. Return a command tuple or ``None``.

        Parameters
        ----------
        obs : r2d3_model_interfaces.msg.Observation
            Latest observation (head camera, joint state, arm states, ...).

        Returns
        -------
        Optional[tuple]
            One of:

            * ``None`` — no command this tick
            * ``("movej", side, joint_targets)`` — joint-space arm command
              ``side`` is ``"left"`` or ``"right"``; ``joint_targets`` is a
              7-tuple of radians
            * ``("movej_p", side, pose)`` — Cartesian-via-joint arm command
            * ``("gripper", side, position)`` — gripper command, position in
              ``1..1000`` (maps to 0–70 mm opening)
            * ``("lift", height_mm, speed_pct)`` — body lift command
        """
        return None

    def on_deactivate(self) -> None:
        """Called when the lifecycle node transitions out of *active*."""

    def on_cleanup(self) -> None:
        """Called when the lifecycle node is cleaned up. Free resources here."""
