"""R2D3 articulation wrapper.

One source of truth for "how do I move this robot from Python?".
Sim-thread-only: ROS callbacks must NOT call into here directly —
they write into a ``LatestCommandCache``; the sim loop flushes the
cache via these methods once per physics step.

Public surface (all units in radians / meters unless noted):

    Robot.initialize()                   bind articulation handle, build
                                         JOINT_INDEX, lock AGV wheels
    Robot.set_arm_targets(side, q)       q is a 7-tuple, side in {'left','right'}
    Robot.set_finger(side, drive_m)      drive_m in [0, FINGER_DRIVE_MAX_M];
                                         writes BOTH drive and mimic joints
                                         (USD's NewtonMimicAPI is a no-op
                                         under PhysX)
    Robot.set_lift_m(height_m)           height_m in [0, LIFT_MAX_M]
    Robot.set_head(pan, tilt)            radians, both clamped to URDF limits
    Robot.get_joint_positions()          dict[str, float]
    Robot.get_lift_m()                   float
"""
from __future__ import annotations

import logging
from typing import Dict, Optional

from . import sim_topics as t

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Canonical home / rest state.
#
# The two arm bases are mounted mirrored (r_joint1 carries an extra pi yaw),
# so IDENTICAL nonzero joint values splay the arms asymmetrically — that was
# the "weird" pose. The mechanical-neutral all-zeros config folds both arms
# compactly and symmetrically beside the lift column, and is the one pose
# that renders reliably (most other configs intermittently drive the
# render-side transforms to NaN — the dropped-joint-damping bug; free pose
# exploration is blocked until that's fixed). Head is neutral (pan/tilt 0).
#
# Lift: the task YAMLs specify body_lift_mm=1200 (1.2 m), but the USD
# platform_joint range is 0-1.0 m and lift near the limit NaN'd; 0.5 m is
# the safe-rendering default until the lift units/range mismatch is tuned.
HOME_LEFT_ARM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME_RIGHT_ARM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME_LIFT_M = 0.5
HOME_HEAD_PAN_RAD = 0.0
HOME_HEAD_TILT_RAD = 0.0
HOME_FINGER_M = t.FINGER_DRIVE_MAX_M    # grippers open

_SIDE_TO_ARM_JOINTS = {
    "left": t.LEFT_ARM_JOINTS,
    "right": t.RIGHT_ARM_JOINTS,
}
_SIDE_TO_FINGER_PAIR = {
    "left":  (t.LEFT_FINGER_DRIVE,  t.LEFT_FINGER_MIMIC),
    "right": (t.RIGHT_FINGER_DRIVE, t.RIGHT_FINGER_MIMIC),
}


class Robot:
    """Bind to the R2D3 articulation and expose a small commanding API."""

    def __init__(self, prim_path: str) -> None:
        self._prim_path = prim_path
        self._art = None
        self._idx: Dict[str, int] = {}

    # ------------------------------------------------------------------ init
    def initialize(self) -> None:
        """Bind the SingleArticulation handle.

        Must be called after ``world.reset()`` has run at least once —
        physics handles aren't valid until the simulation view has been
        created.
        """
        from isaacsim.core.prims import SingleArticulation

        self._art = SingleArticulation(prim_path=self._prim_path)
        self._art.initialize()

        self._idx = {name: self._art.get_dof_index(name)
                     for name in self._art.dof_names}
        logger.info(
            "Robot initialized at %s — %d DOFs total: %s",
            self._prim_path, self._art.num_dof, list(self._idx.keys()),
        )

        missing = [n for n in t.ALL_ACTUATED_JOINTS if n not in self._idx]
        if missing:
            raise RuntimeError(
                f"USD articulation is missing expected joints: {missing}. "
                f"Available: {list(self._idx.keys())}"
            )

        self._configure_drives()

    # --------------------------------------------------------------- drives
    def _configure_drives(self) -> None:
        """Set up stable PhysX position drives and disable gravity.

        The URDF→USD converter authors no joint drives, so joints are free and
        integrate to NaN. We configure per-DOF PD gains + command position
        TARGETS (not teleport). Gravity is disabled on the articulation: the
        real RM75-B controller does gravity compensation, and with gravity ON
        the drive can't hold the chain (diverges in ~70 steps); OFF it is
        stable and tracks targets. Scene objects still fall (gravity is
        per-articulation), and contact force-torque still registers.
        Self-collisions, solver iters, and the joint DriveAPI (stiffness/
        damping) are authored pre-reset in scene.configure_articulation_physics().
        Here we just disable gravity, reassert the gains at runtime (matching
        the authored values), and init the target vector.
        """
        import numpy as np

        self._art.disable_gravity()
        dof = self._art.dof_names
        kps = np.array([t.DRIVE_GAINS[t.drive_group(nm)][0] for nm in dof],
                       dtype=np.float32)
        kds = np.array([t.DRIVE_GAINS[t.drive_group(nm)][1] for nm in dof],
                       dtype=np.float32)
        ctrl = self._art.get_articulation_controller()
        ctrl.switch_control_mode("position")
        ctrl.set_gains(kps=kps, kds=kds)
        # Persistent FULL drive-target vector. A partial apply_action (with
        # joint_indices) clobbers the other DOFs' targets, so every command
        # updates this vector and re-applies the WHOLE thing.
        self._targets = np.asarray(
            self._art.get_joint_positions(), dtype=np.float32
        ).copy()
        logger.info("configured position drives on %d DOFs; gravity disabled",
                    self._art.num_dof)

    # ----------------------------------------------------------------- AGV
    def lock_agv_wheels(self) -> None:
        """Hold the 10 AGV wheel joints at zero with zero velocity.

        The URDF authors the wheels as continuous joints with no damping or
        friction. PhysX simulates them as free spinners, and any tiny
        impulse propagates to the rest of the articulation, eventually
        producing NaN positions across all DOFs. We pin them to zero
        position+velocity by setting their joint state directly.
        """
        import numpy as np

        self._wheel_indices = [self._idx[n] for n in t.AGV_WHEEL_JOINTS
                               if n in self._idx]
        if not self._wheel_indices:
            logger.warning("no AGV wheel joints found to lock; skipping")
            return

        zeros = np.zeros(len(self._wheel_indices), dtype=np.float32)
        self._art.set_joint_positions(zeros, joint_indices=self._wheel_indices)
        self._art.set_joint_velocities(zeros, joint_indices=self._wheel_indices)
        logger.info("zeroed %d AGV wheel joints", len(self._wheel_indices))

    def hold_agv_wheels(self) -> None:
        """Called each sim step to keep the AGV wheels at zero state.

        Forcibly overwrites position AND velocity. Skips if either has gone
        NaN — that's a separate bug to chase, not something to write back
        into the articulation.
        """
        import numpy as np
        if not hasattr(self, "_wheel_indices") or not self._wheel_indices:
            return
        zeros = np.zeros(len(self._wheel_indices), dtype=np.float32)
        self._art.set_joint_positions(zeros, joint_indices=self._wheel_indices)
        self._art.set_joint_velocities(zeros, joint_indices=self._wheel_indices)

    # ------------------------------------------------------------ commanding
    def set_arm_targets(self, side: str, q: list[float]) -> None:
        names = _SIDE_TO_ARM_JOINTS[side]
        if len(q) != len(names):
            raise ValueError(
                f"set_arm_targets({side!r}) expects {len(names)} values, got {len(q)}"
            )
        self._apply(names, q)

    def set_finger(self, side: str, drive_m: float) -> None:
        drive_m = max(0.0, min(t.FINGER_DRIVE_MAX_M, float(drive_m)))
        drive_name, mimic_name = _SIDE_TO_FINGER_PAIR[side]
        self._apply([drive_name, mimic_name], [drive_m, drive_m])

    def set_lift_m(self, height_m: float) -> None:
        height_m = max(0.0, min(t.LIFT_MAX_M, float(height_m)))
        self._apply([t.LIFT_JOINT], [height_m])

    def set_head(self, pan_rad: float, tilt_rad: float) -> None:
        self._apply(t.HEAD_JOINTS, [pan_rad, tilt_rad])

    def go_home(self) -> None:
        """Drive the robot to its canonical home / rest state.

        Lift up, head neutral (slight down-tilt), both arms to the symmetric
        home pose, grippers open. Single source of truth for "home" — used by
        bring_up and the capture tool.
        """
        self.set_lift_m(HOME_LIFT_M)
        self.set_head(HOME_HEAD_PAN_RAD, HOME_HEAD_TILT_RAD)
        self.set_arm_targets("left", HOME_LEFT_ARM)
        self.set_arm_targets("right", HOME_RIGHT_ARM)
        self.set_finger("left", HOME_FINGER_M)
        self.set_finger("right", HOME_FINGER_M)

    # ----------------------------------------------------------------- state
    def get_joint_positions(self) -> Dict[str, float]:
        pos = self._art.get_joint_positions()
        return {name: float(pos[i]) for name, i in self._idx.items()}

    def get_lift_m(self) -> float:
        return float(self._art.get_joint_positions(
            joint_indices=[self._idx[t.LIFT_JOINT]]
        )[0])

    # Wrist link carrying the 6-axis force-torque sensor (between arm and hand).
    _WRIST_LINK = {"left": "l_link7", "right": "r_link7"}

    def get_wrist_wrench(self, side: str):
        """6-axis force-torque at the wrist [fx, fy, fz, tx, ty, tz] (N, N·m).

        Reads the measured joint reaction force at the wrist link — the sim
        analogue of the real RM75-B's integrated wrist FT sensor (Sixforce).
        Returns zeros if the reading is non-finite (the dropped-joint-damping
        workaround means these are holding forces, not clean contact forces,
        until proper dynamics is restored).
        """
        import numpy as np
        try:
            forces = self._art.get_measured_joint_forces()   # (num_links+1, 6)
            idx = self._art.get_link_index(self._WRIST_LINK[side])
            w = np.asarray(forces[idx], dtype=np.float32)
            if w.shape == (6,) and np.all(np.isfinite(w)):
                return w
        except Exception:  # noqa: BLE001
            pass
        return np.zeros(6, dtype=np.float32)

    @property
    def joint_index(self) -> Dict[str, int]:
        return dict(self._idx)  # frozen copy

    @property
    def num_dof(self) -> int:
        return self._art.num_dof if self._art is not None else 0

    # ----------------------------------------------------------------- helpers
    def _apply(self, names: list[str], values: list[float]) -> None:
        """Command position-drive TARGETS for the listed joints.

        Feeds the articulation's PD drives (configured in ``_configure_drives``);
        PhysX integrates the joints to these targets each step. The target
        persists until changed, so a single call holds the pose — no per-step
        re-issue or velocity-zeroing needed.
        """
        from isaacsim.core.utils.types import ArticulationAction

        for nm, v in zip(names, values):
            self._targets[self._idx[nm]] = float(v)
        # Apply the FULL target vector (partial joint_indices would reset the
        # other joints' targets — see _configure_drives).
        self._art.apply_action(ArticulationAction(joint_positions=self._targets))
