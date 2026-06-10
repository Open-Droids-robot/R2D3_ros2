"""Topic / frame / joint name contract between Isaac Sim and the humble bridge.

Isaac publishes / subscribes only standard ROS 2 messages (sensor_msgs,
std_msgs, geometry_msgs) on the names defined here. The companion
`r2d3_humble_bridge/r2d3_humble_bridge/topics.py` mirrors these names and
defines the upstream `rm_ros_interfaces`-flavored topics it bridges to.

This module is the single source of truth on the Isaac side. Importing
a typo at runtime is preferable to a topic name silently mismatching.
"""
from __future__ import annotations

# ---------------------------------------------------------------------------
# Namespace
# ---------------------------------------------------------------------------
NS = "/r2d3/sim"

# ---------------------------------------------------------------------------
# State topics — Isaac publishes, bridge (and anyone else) subscribes
# ---------------------------------------------------------------------------
JOINT_STATES   = f"{NS}/joint_states"          # sensor_msgs/JointState   (all DOFs incl. fingers + wheels)
LIFT_STATE     = f"{NS}/lift_state"            # std_msgs/Float64         (meters)
READY          = f"{NS}/ready"                 # std_msgs/Bool (latched)  (true after first physics step)
EE_POSE_LEFT   = f"{NS}/ee_pose/left"          # geometry_msgs/PoseStamped (l_link7 in base_link_underpan frame)
EE_POSE_RIGHT  = f"{NS}/ee_pose/right"         # geometry_msgs/PoseStamped
WRENCH_LEFT    = f"{NS}/wrench/left"           # geometry_msgs/WrenchStamped (left wrist 6-axis FT)
WRENCH_RIGHT   = f"{NS}/wrench/right"          # geometry_msgs/WrenchStamped (right wrist 6-axis FT)

# ---------------------------------------------------------------------------
# Command topics — bridge publishes, Isaac subscribes
# ---------------------------------------------------------------------------
CMD_LEFT_ARM     = f"{NS}/cmd/left_arm"        # sensor_msgs/JointState   (positions for l_joint1..7)
CMD_RIGHT_ARM    = f"{NS}/cmd/right_arm"       # sensor_msgs/JointState   (positions for r_joint1..7)
CMD_LIFT         = f"{NS}/cmd/lift"            # std_msgs/Float64         (target height in meters)
CMD_LEFT_FINGER  = f"{NS}/cmd/left_finger"     # std_msgs/Float64         (target drive in meters [0, 0.035])
CMD_RIGHT_FINGER = f"{NS}/cmd/right_finger"    # std_msgs/Float64
CMD_HEAD         = f"{NS}/cmd/head"            # sensor_msgs/JointState   (head_joint1, head_joint2) — optional

# ---------------------------------------------------------------------------
# OmniGraph-managed topics (not handled by sim_adapter; configured in sensors.py)
# ---------------------------------------------------------------------------
CLOCK                  = "/clock"
TF                     = "/tf"
TF_STATIC              = "/tf_static"
CAMERA_COLOR_IMAGE     = "/camera/color/image_raw"          # head D435
CAMERA_DEPTH_IMAGE     = "/camera/depth/image_raw"
CAMERA_COLOR_INFO      = "/camera/color/camera_info"
CAMERA_DEPTH_INFO      = "/camera/depth/camera_info"
# wrist (in-hand) D435s, one per arm
L_WRIST_COLOR_IMAGE    = "/l_wrist_camera/color/image_raw"
L_WRIST_DEPTH_IMAGE    = "/l_wrist_camera/depth/image_raw"
L_WRIST_COLOR_INFO     = "/l_wrist_camera/color/camera_info"
L_WRIST_DEPTH_INFO     = "/l_wrist_camera/depth/camera_info"
R_WRIST_COLOR_IMAGE    = "/r_wrist_camera/color/image_raw"
R_WRIST_DEPTH_IMAGE    = "/r_wrist_camera/depth/image_raw"
R_WRIST_COLOR_INFO     = "/r_wrist_camera/color/camera_info"
R_WRIST_DEPTH_INFO     = "/r_wrist_camera/depth/camera_info"

# ---------------------------------------------------------------------------
# Frame IDs (TF)
# ---------------------------------------------------------------------------
BASE_FRAME              = "base_link_underpan"
LEFT_EE_FRAME           = "l_link7"
RIGHT_EE_FRAME          = "r_link7"
CAMERA_LINK_FRAME       = "head_camera_link"
CAMERA_COLOR_OPT_FRAME  = "head_camera_color_optical_frame"
CAMERA_DEPTH_OPT_FRAME  = "head_camera_depth_optical_frame"
L_WRIST_COLOR_OPT_FRAME = "l_wrist_camera_color_optical_frame"
L_WRIST_DEPTH_OPT_FRAME = "l_wrist_camera_depth_optical_frame"
R_WRIST_COLOR_OPT_FRAME = "r_wrist_camera_color_optical_frame"
R_WRIST_DEPTH_OPT_FRAME = "r_wrist_camera_depth_optical_frame"

# ---------------------------------------------------------------------------
# Joint names (must match USD authoring at /r2d3_v1/Physics/*)
# ---------------------------------------------------------------------------
LEFT_ARM_JOINTS  = [f"l_joint{i}" for i in range(1, 8)]   # 7 DOF
RIGHT_ARM_JOINTS = [f"r_joint{i}" for i in range(1, 8)]   # 7 DOF
HEAD_JOINTS      = ["head_joint1", "head_joint2"]
LIFT_JOINT       = "platform_joint"                       # prismatic, 0..1 m

# ---------------------------------------------------------------------------
# End-effector selection. R2D3_EE = "dexterous" (Inspire RH56 hand, default) or
# "gripper" (2-finger parallel gripper). Selects the USD variant + the actuated
# hand/finger joints. Set R2D3_EE BEFORE importing r2d3_sim (the grasp demo's
# --ee flag does this); a USD per variant is built by scripts/build_robot.sh.
# ---------------------------------------------------------------------------
import os
EE_TYPE = os.environ.get("R2D3_EE", "dexterous").strip().lower()
if EE_TYPE not in ("dexterous", "gripper"):
    EE_TYPE = "dexterous"

# Inspire RH56 dexterous hand: 12 revolute joints per side (prefix l_dex_/r_dex_).
# The 4 "proximal" flex joints + the thumb pitch/yaw are the actuated DOFs; the
# "intermediate" + thumb distal follow (driven together here for a power grasp).
_HAND_JOINT_SUFFIXES = [
    "thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint",
    "thumb_intermediate_joint", "thumb_distal_joint",
    "index_proximal_joint", "index_intermediate_joint",
    "middle_proximal_joint", "middle_intermediate_joint",
    "ring_proximal_joint", "ring_intermediate_joint",
    "pinky_proximal_joint", "pinky_intermediate_joint",
]
if EE_TYPE == "gripper":
    # parallel gripper: 2 prismatic finger joints per side (drive + mimic)
    LEFT_HAND_JOINTS = ["l_finger_drive", "l_finger_mimic"]
    RIGHT_HAND_JOINTS = ["r_finger_drive", "r_finger_mimic"]
else:
    LEFT_HAND_JOINTS = [f"l_dex_{s}" for s in _HAND_JOINT_SUFFIXES]
    RIGHT_HAND_JOINTS = [f"r_dex_{s}" for s in _HAND_JOINT_SUFFIXES]

# AGV — locked at init time, never commanded after that
AGV_WHEEL_JOINTS = [
    "joint_left_wheel",
    "joint_right_wheel",
    "joint_swivel_wheel_1_1", "joint_swivel_wheel_1_2",
    "joint_swivel_wheel_2_1", "joint_swivel_wheel_2_2",
    "joint_swivel_wheel_3_1", "joint_swivel_wheel_3_2",
    "joint_swivel_wheel_4_1", "joint_swivel_wheel_4_2",
]

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------
FINGER_DRIVE_MAX_M = 0.035       # matches parallel_gripper.urdf.xacro limit
LIFT_MAX_M         = 1.0         # matches platform_joint URDF range (0..1 m)

# ---------------------------------------------------------------------------
# Position-drive PD gains per joint group (stiffness, damping). The converter
# authors NO joint drives, so these must be authored as UsdPhysics.DriveAPI
# (scene.configure_articulation_physics) for the joints to actually track
# targets. Tuned with gravity disabled on the articulation.
#   angular drives (revolute): stiffness in N·m/rad, damping N·m·s/rad
#   linear drive  (prismatic lift/fingers): N/m, N·s/m
# ---------------------------------------------------------------------------
DRIVE_GAINS = {
    "arm":    (2000.0, 200.0),
    "head":   (400.0, 40.0),
    "lift":   (80000.0, 8000.0),
    "hand":   (30.0, 3.0),       # light dexterous-hand fingers (revolute, N·m/rad)
    "finger": (400.0, 40.0),     # parallel-gripper fingers (prismatic, N/m)
    "wheel":  (200.0, 20.0),     # AGV wheels in the "mobile" build (revolute)
}
DRIVE_MAX_FORCE = 1.0e6          # generous; don't let maxForce limit tracking


def drive_group(name: str) -> str:
    if name.startswith(("l_joint", "r_joint")):
        return "arm"
    if name.startswith("head_joint"):
        return "head"
    if name == LIFT_JOINT:
        return "lift"
    if "wheel" in name:
        return "wheel"    # AGV wheels (revolute in the "mobile" build)
    if "finger" in name:
        return "finger"   # parallel-gripper prismatic finger joints
    return "hand"   # l_dex_* / r_dex_* dexterous-hand revolute joints

ALL_ACTUATED_JOINTS = (
    LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + HEAD_JOINTS + [LIFT_JOINT] +
    LEFT_HAND_JOINTS + RIGHT_HAND_JOINTS
)  # 7+7+2+1 + 12+12 = 41 DOFs
