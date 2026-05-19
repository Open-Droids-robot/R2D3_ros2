"""Placeholder lifecycle test. Runs inside the Humble container; skipped on host.

Once the dependent interfaces packages are built, this exercises:
  - construct R2D3Model
  - drive transitions: unconfigured -> inactive -> active -> inactive -> unconfigured
  - verify command publishers are inactive while not in 'active' state
  - verify RunTask goal is rejected in non-active states
"""

import pytest


@pytest.mark.skip(reason="requires ROS 2 + interface packages built; runs in Humble container")
def test_lifecycle_smoke():
    # Intentionally empty — kept as a hook for the future CI matrix.
    pass
