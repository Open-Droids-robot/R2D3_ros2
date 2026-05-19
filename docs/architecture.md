# R2D3 Isaac Sim — System Architecture (V1)

> Locked: 2026-05-19 (`isaac-sim-v1` branch). Design informed by the [`intrinsic-dev/aic`](https://github.com/intrinsic-dev/aic) hackathon template the project primer named as our reference.

## Goals

- A simulated **R2D3** (dual 7-DOF arm + parallel grippers + head-mounted D435 + body lift + AGV chassis) running in **NVIDIA Isaac Sim 5.1**.
- A **deterministic evaluation harness** that loads YAML task definitions, runs a participant policy, and emits a tiered score.
- **Same ROS2 topic/message surface as the real R2D3** (`rm_ros_interfaces`) — so policies port between sim and hardware with zero API changes.
- Participants ship a **container with a ROS 2 Lifecycle node** as their submission.

## Components

```
┌────────────────────────────── host (Ubuntu 24.04, RTX 4090) ─────────────────────────────┐
│                                                                                          │
│  ┌──────────────────────────────────┐   ┌──────────────────────────────────────────────┐ │
│  │ Isaac Sim 5.1 container          │   │ ROS 2 Humble side (separate container or     │ │
│  │   image: r2d3-isaac-sim:dev      │   │   colocated; bridge talks DDS on host net)   │ │
│  │                                  │   │                                              │ │
│  │  ┌────────────────────────────┐  │   │  ┌──────────────────────────────────────┐    │ │
│  │  │ isaac_sim/r2d3_sim/        │  │   │  │ r2d3_model  (Lifecycle Node)         │    │ │
│  │  │   scene.py    world+assets │  │   │  │   ↳ participant submission           │    │ │
│  │  │   robot.py    R2D3 art.    │  │   │  │   subscribes: /r2d3/observations     │    │ │
│  │  │   sensors.py  D435 wrap    │  │   │  │   publishes: /right_arm_controller/  │    │ │
│  │  │   bridge.py   ROS2 conf    │  │◄──┼──┤              rm_driver/*             │    │ │
│  │  └────────────────────────────┘  │   │  └──────────────────────────────────────┘    │ │
│  │                                  │   │                                              │ │
│  │  ┌────────────────────────────┐  │   │  ┌──────────────────────────────────────┐    │ │
│  │  │ r2d3_gripper_bridge        │  │   │  │ r2d3_eval  (CLI)                     │    │ │
│  │  │   maps Gripperset (1-1000) │  │   │  │   loads tasks/<slug>.yaml            │    │ │
│  │  │   → finger prismatic joints│  │   │  │   drives r2d3_engine                 │    │ │
│  │  └────────────────────────────┘  │   │  │   emits JSON report                  │    │ │
│  │                                  │   │  └──────────────────────────────────────┘    │ │
│  │  ┌────────────────────────────┐  │   │                                              │ │
│  │  │ r2d3_engine                │  │   │  ┌──────────────────────────────────────┐    │ │
│  │  │   trial state machine      │  │   │  │ r2d3_scoring                         │    │ │
│  │  │   spawn scene, reset robot │  │◄──┼──┤   Tier 1 (liveness) + Tier 2 (task)  │    │ │
│  │  │   send RunTask action      │  │   │  └──────────────────────────────────────┘    │ │
│  │  └────────────────────────────┘  │   │                                              │ │
│  └──────────────────────────────────┘   └──────────────────────────────────────────────┘ │
│                  ▲                                                                       │
│                  │ GPU 1 (pinned)                                                        │
│            ┌─────┴───────┐                                                               │
│            │  RTX 4090   │                                                               │
│            └─────────────┘                                                               │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```

Note that `r2d3_engine`, `r2d3_scoring`, `r2d3_gripper_bridge` start as Python modules under `isaac_sim/r2d3_sim/` and may graduate to standalone ROS2 packages later. **`r2d3_model` is a separate package by design** — it's the submission boundary.

## Repository layout (V1)

```
r2d3_isaac/                            # = Open-Droids-robot/R2D3_ros2 @ isaac-sim-v1
├── isaac_sim/                         # Container-only Python (imports omni.*, isaacsim.*)
│   ├── r2d3_sim/                      # Scene/robot/sensor wrappers + engine + bridges
│   │   ├── scene.py                   # World assembly, asset loading
│   │   ├── robot.py                   # R2D3 articulation (arms, lift, grippers)
│   │   ├── sensors.py                 # D435 publishers
│   │   ├── bridge.py                  # ROS2 graph wiring (Isaac Sim ROS2 bridge config)
│   │   ├── gripper_bridge.py          # Gripperset (1-1000) -> finger joint commands
│   │   ├── engine.py                  # Trial state machine
│   │   └── scoring.py                 # Tier 1 + Tier 2 evaluators
│   ├── usd/                           # R2D3 USD asset (output of URDF -> USD pipeline)
│   ├── scenes/                        # One scene script per task (loads task YAML)
│   └── tests/
├── tasks/                             # Task YAML configs (consumed by r2d3_eval)
│   ├── SCHEMA.md                      # YAML key reference
│   ├── pick_and_place.yaml
│   ├── stacking.yaml
│   ├── bimanual_handoff.yaml
│   └── vision_guided_grasp.yaml
├── r2d3_eval/                         # CLI (host-side, runs outside the Isaac container)
│   ├── __init__.py
│   ├── cli.py                         # `r2d3-eval <task> --policy <image>` entry
│   └── report.py                      # JSON report writer
├── r2d3_model/                        # Submission template (separate ROS2 package)
│   ├── package.xml
│   ├── setup.py
│   └── r2d3_model/
│       ├── r2d3_model.py              # LifecycleNode handling RunTask action
│       └── policy.py                  # Stub policy class participants override
├── Docker/                            # Existing upstream Docker/docker/* untouched
│   ├── docker/                        # ROS2 Foxy/Humble/Jazzy (upstream)
│   └── isaac/                         # Our Isaac Sim 5.1 image + compose
├── ros2_rm_robot/                     # Existing arm packages (untouched)
├── ros2_realsense2/                   # Existing camera packages (untouched)
├── ros2_servo_driver/                 # (untouched)
├── ros2_total_demo/                   # (untouched, kept as reference)
├── ros2_agv_robot/                    # (untouched, dormant for V1)
├── docs/
├── scripts/
└── .vscode/
```

## Trial lifecycle (one task execution)

Mirrors the AIC engine state machine. The engine drives this; the participant model is reactive.

```
                         engine                                participant model
                         ──────                                ─────────────────
       Uninitialized   ──┐
                         │
                         │ check lifecycle node discoverable
                         │ (and in `unconfigured` state)
                         │ – validates: model is not moving
                         │   the robot pre-activation
                         ▼
       ModelReady      ──┐
                         │
                         │ spawn scene per YAML, reset robot
                         │ to home, set body lift
                         ▼
       EndpointsReady  ──┐
                         │
                         │ check all topics/services up
                         │   /r2d3/observations            (publisher: r2d3_adapter)
                         │   /right_arm_controller/...     (subscriber: r2d3_model)
                         │   /left_arm_controller/...      (subscriber: r2d3_model)
                         │   /run_task action server
                         │   /reset_joints service
                         ▼
       SimulatorReady  ──┐
                         │ transition model: configure → activate
                         │                                          on_configure
                         │                                          on_activate
                         ▼                                          (model becomes ACTIVE)
       ScoringReady    ──┐
                         │ start Tier 1 monitor (msg rate, latency)
                         │ start Tier 2 monitor (TF poses, contacts, FT)
                         ▼
       TaskStarted     ──┐
                         │ send RunTask goal ─────────────────────► action callback
                         │                                          policy.run() loop:
                         │                                            recv Observation
                         │                                            compute action
                         │                                            publish Movej / Gripperset
                         │                                            ...
                         │                                          send Result(success, msg)
                         ▼
       TaskCompleted   ──┐
                         │ stop Tier 1/2 monitors; compute scores
                         │ deactivate + cleanup model
                         │                                          on_deactivate
                         │                                          on_cleanup
                         │ remove spawned entities
                         ▼
       Completed
```

## ROS 2 graph

### Reused from `rm_ros_interfaces` (upstream)
| Direction | Topic | Type |
|---|---|---|
| model → controller | `/{left,right}_arm_controller/rm_driver/movej_p_cmd` | `rm_ros_interfaces/Movejp` |
| model → controller | `/{left,right}_arm_controller/rm_driver/movej_cmd` | `rm_ros_interfaces/Movej` |
| model → controller | `/{left,right}_arm_controller/rm_driver/set_gripper_position_cmd` | `rm_ros_interfaces/Gripperset` |
| model → controller | `/{left,right}_arm_controller/rm_driver/set_gripper_pick_cmd` | `rm_ros_interfaces/Gripperpick` |
| model → controller | `/left_arm_controller/rm_driver/set_lift_height_cmd` | `rm_ros_interfaces/Liftheight` |
| controller → model | `/{left,right}_arm_controller/rm_driver/get_current_arm_state_result` | `rm_ros_interfaces/Armstate` |
| controller → model | `/{left,right}_arm_controller/rm_driver/movej_p_result` | `std_msgs/Bool` |
| sensors → model | `/camera/color/image_raw` | `sensor_msgs/Image` |
| sensors → model | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| sensors → model | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| tf | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` |

### New (our additions, in `r2d3_model_interfaces` and `r2d3_task_interfaces`)
| Direction | Topic / Service / Action | Type |
|---|---|---|
| adapter → model | `/r2d3/observations` | `r2d3_model_interfaces/Observation` |
| engine → model | `/run_task` | `r2d3_task_interfaces/action/RunTask` |
| engine → controller | `/reset_joints` | `r2d3_engine_interfaces/srv/ResetJoints` |
| scoring → engine | `/r2d3/scoring/tier1_state`, `/tier2_state` | `r2d3_scoring_interfaces/TierState` |

### `Observation.msg` (R2D3-specific, AIC-inspired)

```
# r2d3_model_interfaces/Observation.msg
std_msgs/Header header

# Head camera (the only physical D435 on V1 hardware)
sensor_msgs/Image       head_color
sensor_msgs/Image       head_depth
sensor_msgs/CameraInfo  head_camera_info

# Joint state (both arms + lift + head pan/tilt)
sensor_msgs/JointState  joint_states

# Per-arm state
rm_ros_interfaces/Armstate  left_arm_state
rm_ros_interfaces/Armstate  right_arm_state

# Optional force-torque (if the task involves contact, otherwise zeros)
geometry_msgs/WrenchStamped left_wrench
geometry_msgs/WrenchStamped right_wrench

# Body lift state (height in mm)
rm_ros_interfaces/Liftstate lift_state
```

### `RunTask.action` (generic over the 4 task types)

```
# Goal
string  task_id          # slug; e.g. "pick_and_place"
uint64  task_seed        # for deterministic scene spawn
uint32  trial_index      # 0..N for multiple runs of same task

---
# Result
bool    success
float64 tier1_score
float64 tier2_score
float64 total_score      # weighted combination per task YAML
string  message
string  metrics_json     # full breakdown for the leaderboard

---
# Feedback
string  phase            # "setup" | "running" | "scoring"
float32 progress         # 0.0 – 1.0 if estimable
```

Rationale: AIC has one action *per task type* because they only have one task. We have four; a single generic action with `task_id` keeps the interface package small.

## Gripper joint model

The real R2D3's parallel grippers expose a controller-level API (`Gripperset`), not joint-level (see [URDF audit](urdf_audit.md)). For Isaac Sim we still need physical fingers to simulate contact. Author each gripper in the USD asset as:

```
Parent link: {l,r}_hand_base_link
Joints:
  {l,r}_finger_drive   prismatic
                       axis  = +X (away from palm centerline)
                       range = 0.0 – 0.035 m
                       velocity ≤ 0.05 m/s
                       effort ≈ 100 N    # estimate; revise from RealMan datasheet
  {l,r}_finger_mimic   prismatic, mimic of *_finger_drive with multiplier = -1
```

A small Python node (`isaac_sim/r2d3_sim/gripper_bridge.py`) subscribes to **the existing** `Gripperset` topic from `rm_ros_interfaces` and maps:

```
position ∈ [1, 1000]  →  drive_joint_target = (position / 1000) × 0.035
```

So policies that work on the real robot work in sim without code changes. The mimic constraint guarantees symmetric closure.

## D435 mounting

Single head-mounted RealSense D435 — **matches real hardware** (verified from `pic/dual_lift_robot.jpg`):

| Parameter | Value |
|---|---|
| Parent link | `head_link2` (after head pan + tilt) |
| Position | `+0.05 m` along the head's local +X (forward) |
| Orientation | `roll=0, pitch=0, yaw=0` (forward-facing) |
| Source xacro | `ros2_realsense2/realsense2_description/urdf/_d435.urdf.xacro` (existing macro, `use_nominal_extrinsics=false`) |
| Topics | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`, `/camera/depth/camera_info` |

**No wrist cameras for V1.** The real hardware doesn't have them; adding them would let participants over-fit a sensor topology that doesn't transfer. Defer to V2 if a future task requires close-up sensing.

## AGV chassis (V1 treatment)

The 75b URDF includes the Woosh AGV (10 continuous wheel joints + `base_link_underpan` + caster links). For V1 we want a **stationary base**:

- **Keep the AGV in the URDF/USD** (don't strip — preserves V2 upgrade path)
- **Lock wheel joints** in the Isaac scene (`drive_mode = OFF`, `joint_limit_lower = joint_limit_upper = 0`)
- **Disable physics on the AGV chassis** (`articulation_enabled=False` for the AGV subtree) so the lift/torso reaction forces don't unintentionally roll it

This is reversible — V2 just re-enables the joints and adds a controller.

## URDF → USD conversion

Default tool: **Isaac Sim's built-in `urdf_importer` extension** (`omni.importer.urdf`). Workflow:

1. Render the xacro to URDF (already done — `dual_rm_75b_description.urdf` exists in tree)
2. Compose a V1-only wrapper xacro that includes the 75b URDF + the `sensor_d435` macro mounted on `head_link2` + the gripper finger sub-assembly
3. Re-render to a flat URDF
4. Import via `urdf_importer` (Python API: `omni.importer.urdf.cmds.URDFParseAndImportFile`)
5. Save USD to `isaac_sim/usd/r2d3.usd`
6. In Isaac Sim, manually verify scale, axes, link colors, finger contact shapes
7. Commit USD (if size permits) or set up git-lfs

## Open questions / V2 candidates

- **Gripper effort** — 100 N is an estimate; refine from RealMan datasheet when available.
- **Head pan/tilt commands** — the URDF has these joints, but the demo scripts don't drive them. Do V1 tasks need a head controller, or can we leave the head static?
- **Force-torque sensors** — the real robot has 6-axis FT via `Sixforce.msg`. Does Isaac Sim's articulation API publish wrist wrench out of the box, or do we need to attach a force sensor manually?
- **Multi-camera support** — defer to V2 along with wrist cameras and dexterous hands.
- **Domain randomization** — V1 task YAMLs include `spawn` with uniform distributions; richer randomization (textures, lighting) goes in V2.
- **Leaderboard** — out of scope for V1 (data format is fixed in `RunTask` result; web UI is V2).
