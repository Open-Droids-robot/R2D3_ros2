# R2D3 examples

Runnable demos of the `R2D3` platform SDK. Each is launched through the Isaac
wrapper (which sets the env vars + library paths):

```bash
scripts/isaacsim_ros2.sh isaac_sim/examples/01_hello_robot.py
```

| Example | Shows |
|---|---|
| `01_hello_robot.py` | boot, IK arm move, gripper/head/lift, read state |
| `02_cameras.py` | grab head + wrist RGB/depth as numpy, save PNGs |
| `03_teleop.py` | stream target poses with `TeleopServer` |
| `04_rl_env.py` | a Gymnasium rollout in `R2D3Env` (needs `pip install gymnasium`) |
| `05_vlm_loop.py` | perception → action loop (`PerceptionLoop`) with a stub policy |
| `06_drive_base.py` | drive the mobile base (wheels rolling) — needs the `usd_mobile` build |
| `07_grasp_cube.py` | full IK grasp-and-lift of a cube |

## Switch the arm / end-effector

Every example takes **`--ee dexterous|gripper`** to switch between the 5-finger
Inspire hand and the 2-finger parallel gripper — the IK, grasp geometry, and
finger control all adapt automatically:

```bash
scripts/isaacsim_ros2.sh isaac_sim/examples/07_grasp_cube.py --ee dexterous
scripts/isaacsim_ros2.sh isaac_sim/examples/07_grasp_cube.py --ee gripper
```

Both grasp-and-lift the cube (verified: dexterous +0.21 m, gripper +0.21 m). In
your own code it's just the `R2D3(end_effector=...)` argument; the choice is
fixed per process (it selects the USD + Lula description at import), so construct
a new `R2D3` to switch within one script.

The SDK lives in `isaac_sim/r2d3_sim/` (`from isaac_sim.r2d3_sim import R2D3`).
See `docs/api.md` for the full reference and `docs/platform.md` for the
RL / VLM / teleop patterns.
