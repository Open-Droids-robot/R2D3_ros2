"""Topic-name + joint-name constants for the bridge.

MUST stay in sync with `isaac_sim/r2d3_sim/sim_topics.py`. Where the
constants overlap (the `/r2d3/sim/*` namespace and joint name lists),
that file is the authoritative source — we duplicate here because the
Isaac side runs in a different Python env and isn't importable from
ros_humble.
"""
from __future__ import annotations

# ---------------------------------------------------------------------------
# Upstream (participant-facing) namespace — must match r2d3_model
# (`r2d3_model/r2d3_model/r2d3_model.py:93-121`).
# ---------------------------------------------------------------------------

LEFT_ARM_NS  = "/left_arm_controller/rm_driver"
RIGHT_ARM_NS = "/right_arm_controller/rm_driver"

# Commands the bridge subscribes to.
MOVEJ_LEFT       = f"{LEFT_ARM_NS}/movej_cmd"
MOVEJ_RIGHT      = f"{RIGHT_ARM_NS}/movej_cmd"
MOVEJP_LEFT      = f"{LEFT_ARM_NS}/movej_p_cmd"
MOVEJP_RIGHT     = f"{RIGHT_ARM_NS}/movej_p_cmd"
GRIPPERSET_LEFT  = f"{LEFT_ARM_NS}/set_gripper_position_cmd"
GRIPPERSET_RIGHT = f"{RIGHT_ARM_NS}/set_gripper_position_cmd"
LIFT_HEIGHT      = f"{LEFT_ARM_NS}/set_lift_height_cmd"

# State the bridge publishes upstream.
ARMSTATE_LEFT  = f"{LEFT_ARM_NS}/get_current_arm_state_result"
ARMSTATE_RIGHT = f"{RIGHT_ARM_NS}/get_current_arm_state_result"
LIFTSTATE      = f"{LEFT_ARM_NS}/get_lift_state_result"

# Aggregated participant-facing observation.
OBSERVATIONS = "/r2d3/observations"

# ---------------------------------------------------------------------------
# Sim-side namespace (matches isaac_sim/r2d3_sim/sim_topics.py).
# ---------------------------------------------------------------------------

SIM_NS = "/r2d3/sim"

# Isaac publishes:
SIM_JOINT_STATES = f"{SIM_NS}/joint_states"
SIM_LIFT_STATE   = f"{SIM_NS}/lift_state"
SIM_READY        = f"{SIM_NS}/ready"
SIM_EE_LEFT      = f"{SIM_NS}/ee_pose/left"
SIM_EE_RIGHT     = f"{SIM_NS}/ee_pose/right"
SIM_WRENCH_LEFT  = f"{SIM_NS}/wrench/left"
SIM_WRENCH_RIGHT = f"{SIM_NS}/wrench/right"

# Head D435 (published by Isaac's OmniGraph ROS2 bridge on standard topics):
CAMERA_COLOR_IMAGE = "/camera/color/image_raw"
CAMERA_DEPTH_IMAGE = "/camera/depth/image_raw"
CAMERA_COLOR_INFO  = "/camera/color/camera_info"

# Isaac subscribes:
SIM_CMD_LEFT_ARM     = f"{SIM_NS}/cmd/left_arm"
SIM_CMD_RIGHT_ARM    = f"{SIM_NS}/cmd/right_arm"
SIM_CMD_LIFT         = f"{SIM_NS}/cmd/lift"
SIM_CMD_LEFT_FINGER  = f"{SIM_NS}/cmd/left_finger"
SIM_CMD_RIGHT_FINGER = f"{SIM_NS}/cmd/right_finger"

# ---------------------------------------------------------------------------
# Joint names (must match the USD authoring).
# ---------------------------------------------------------------------------

LEFT_ARM_JOINTS  = [f"l_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"r_joint{i}" for i in range(1, 8)]
LIFT_JOINT       = "platform_joint"

# ---------------------------------------------------------------------------
# Physical constants.
# ---------------------------------------------------------------------------

# Gripperset.position is uint16 in [1, 1000] mapping to a 0-70 mm jaw opening
# (35 mm per finger). The Isaac sim_adapter subscribes Float64 in meters,
# but only one finger's drive distance ([0, 0.035] m). The gripper bridge
# does the division.
FINGER_DRIVE_MAX_M = 0.035
GRIPPER_POSITION_MAX = 1000

LIFT_MAX_MM = 2600
LIFT_MAX_M  = 1.0   # USD platform_joint range; Liftheight allows up to 2600 mm
                    # but the USD is authored to 1 m; clamp the bridge there
                    # for V1.
