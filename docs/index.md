# R2D3 Isaac Sim Platform

A port of Open Droids' R2D3 dual-arm composite-lifting mobile manipulator from
Gazebo to NVIDIA Isaac Sim 6.0 — packaged as a platform you can drive from Python
(RL, VLMs, teleop, scripted control) or over ROS.

```python
from isaac_sim.r2d3_sim import R2D3

with R2D3(end_effector="dexterous") as sim:
    sim.reset()
    sim.set_arm_pose("left", [0.45, -0.21, 0.55], sim.top_down_quat)   # IK
    sim.set_gripper("left", 1.0)
    rgb = sim.get_image("head")        # numpy HxWx3, in-process
```

## Get started
- **[Setup](setup.md)** — environments + install (needs `isaacsim[all,extscache]==6.0.0.0`)
- **[Run](run.md)** — launch the examples, verify the install, build the USD
- **[API reference](api.md)** — the `R2D3` SDK: control + sensing surface

## Build on it
- **[Platform guide](platform.md)** — RL (`R2D3Env`), VLM (`PerceptionLoop`), teleop (`TeleopServer`), custom task scenes
- **[Training scenes](scenes.md)** — warehouse / kitchen / living room + recommended environments
- **[Examples](examples.md)** — 7 runnable demos in `isaac_sim/examples/`

## Under the hood
- **[Architecture](architecture.md)** — design, dynamics, gripper-mimic decisions
- **[ROS bridge](bridge.md)** — the two-process custom-message path (`/r2d3/sim/*`, `/r2d3/observations`)
- **[ROS 2 packages](ros2_packages.md)** — the upstream real-robot driver stack
- **[URDF → USD](urdf_to_usd.md)** · **[URDF audit](urdf_audit.md)** — the asset pipeline
- **[Roadmap](roadmap.md)** — what's V1 vs V2

For AI coding agents: **[AGENTS.md](../AGENTS.md)** (run, control, swap parts, build, verify).

## Capabilities (V1)
Dual 7-DOF arms · switchable Inspire 5-finger hand / parallel gripper · body lift ·
head pan/tilt · head + dual wrist D435 cameras (RGB + depth, in-process or ROS) ·
wrist force-torque · Lula IK (left arm) · IK grasp-and-lift · kinematic mobile base.
