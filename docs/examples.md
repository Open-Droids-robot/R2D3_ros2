# Examples

Runnable demos of the platform SDK, in `isaac_sim/examples/`. Launch each through
the Isaac wrapper:

```bash
scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
```

| # | Script | Shows | Notes |
|---|---|---|---|
| 01 | `01_hello_robot.py` | boot, IK arm move, gripper/head/lift, read state | start here |
| 02 | `02_cameras.py` | head + wrist RGB/depth → numpy → PNG | writes to `isaac_sim/tests/captures/` |
| 03 | `03_teleop.py` | stream target poses with `TeleopServer` | swap in your input device |
| 04 | `04_rl_env.py` | a Gymnasium rollout in `R2D3Env` | needs `pip install gymnasium` |
| 05 | `05_vlm_loop.py` | perception → action loop (`PerceptionLoop`) | stub policy; plug in a model |
| 06 | `06_drive_base.py` | drive the mobile base, wheels rolling | needs the `usd_mobile` build |
| 07 | `07_grasp_cube.py` | full IK grasp-and-lift of a cube | uses the `setup` hook + a weld |

Full demos with GIF capture (the polished, lower-level versions) live in
`isaac_sim/tests/`: `grasp_lift_ik.py`, `move_task.py`, `capture_cameras.py`,
`realtime_camera_demo.py`. One-off probes have been archived under
`isaac_sim/tests/diagnostics/`.

See [api.md](api.md) for the SDK reference and [platform.md](platform.md) for the
RL / VLM / teleop patterns these examples instantiate.
