"""Inverse / forward kinematics for the R2D3 LEFT arm (Lula).

Wraps ``LulaKinematicsSolver`` and the per-end-effector URDF + description
selection that the examples used to hard-code. LEFT arm only: the upstream Lula
descriptions are ``l_joint1..7`` (cspace), and the right arm is mounted with an
extra pi yaw — right-arm IK needs a mirrored description yaml that doesn't exist
yet, so the facade fails loud for ``side="right"`` instead of solving wrong.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from . import sim_topics as t
from . import helpers as h

_URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"

# end_effector -> (urdf, lula description yaml). The mesh-free *_lula.urdf are
# committed + portable (see scripts/make_lula_urdf.py); Lula needs only the joint
# tree + the .yaml collision spheres, not the meshes.
_CFG = {
    "dexterous": ("r2d3_v1_dexterous_lula.urdf", "r2d3_left_arm_lula.yaml"),
    "gripper":   ("r2d3_v1_gripper_lula.urdf",   "r2d3_left_arm_lula_gripper.yaml"),
}


class ArmIK:
    """Lula IK for the left arm (default frame ``l_hand_link``)."""

    def __init__(self, end_effector: Optional[str] = None, ee_frame: str = "l_hand_link"):
        from isaacsim.robot_motion.motion_generation.lula import LulaKinematicsSolver
        ee = end_effector or t.EE_TYPE
        urdf, desc = _CFG.get(ee, _CFG["dexterous"])
        self.ee_frame = ee_frame
        self._solver = LulaKinematicsSolver(
            robot_description_path=str(_URDF_DIR / desc),
            urdf_path=str(_URDF_DIR / urdf),
        )

    def sync_base(self, base_pos=None, base_quat_wxyz=None) -> None:
        """Tell the solver where the robot base is — position AND orientation. With no
        args, reads the ``base_link_underpan`` world pose from the stage; reading the
        orientation matters when the mobile base is rotated (a fixed identity quat
        would make IK solve in the wrong frame, e.g. a robot facing -X)."""
        if base_pos is None or base_quat_wxyz is None:
            p, qb = h.world_pose("base_link_underpan")
            base_pos = p if base_pos is None else base_pos
            base_quat_wxyz = qb if base_quat_wxyz is None else base_quat_wxyz
        self._solver.set_robot_base_pose(np.asarray(base_pos, dtype=float),
                                         np.asarray(base_quat_wxyz, dtype=float))

    def ik(self, position, quat_wxyz=None, *, pos_tol: float = 0.01, ori_tol: float = 0.2):
        """Solve IK for the EE frame at a world target. Returns (q[7]|None, ok)."""
        q, ok = self._solver.compute_inverse_kinematics(
            self.ee_frame,
            np.asarray(position, dtype=float),
            None if quat_wxyz is None else np.asarray(quat_wxyz, dtype=float),
            position_tolerance=pos_tol,
            orientation_tolerance=ori_tol,
        )
        return (np.asarray(q, dtype=float) if ok else None), bool(ok)

    def fk(self, q):
        """Forward kinematics for the EE frame -> (xyz, quat_wxyz)."""
        pos, rot = self._solver.compute_forward_kinematics(self.ee_frame, np.asarray(q, dtype=float))
        return np.asarray(pos, dtype=float), h.mat_to_quat(rot)

    @property
    def frames(self):
        return list(self._solver.get_all_frame_names())
