"""Idle policy — never commands the robot. Used as the default for smoke tests.

This is the policy the R2D3Model lifecycle node loads when no `policy` parameter
is set. It does nothing useful — its purpose is to let the engine validate
lifecycle transitions and action-server liveness without involving any task
logic.
"""
from __future__ import annotations

from typing import Optional

from r2d3_model.policy import Policy


class Idle(Policy):
    """No-op policy. Every hook is a stub; step() always returns None."""

    def on_configure(self) -> None:
        pass

    def on_activate(self) -> None:
        pass

    def on_task(self, task_id: str, task_seed: int, trial_index: int) -> None:
        pass

    def step(self, obs) -> Optional[tuple]:
        return None

    def on_deactivate(self) -> None:
        pass

    def on_cleanup(self) -> None:
        pass
