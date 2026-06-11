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
| `07_grasp_cube.py` | full IK grasp-and-lift of a cube (`--ee dexterous\|gripper`) |
| `08_kitchen_manipulation.py` | pick a mug off the kitchen island in a full training scene |
| `09_kitchen_clear_island.py` | **ML-perception pick-and-place** — OWL-ViT finds the mug, robot picks + places it |

`08` loads the **kitchen training scene** (`scenes.load`) — counters/island with a
mug, bowl, and groceries on them — and grasps the mug. See [`../../docs/scenes.md`](../../docs/scenes.md)
for the warehouse / kitchen / living-room scenes + their manipulable objects.

`09` adds **ML perception**: an open-vocabulary detector (OWL-ViT) localises the mug
in the head camera, the pixel is unprojected to 3D via the depth image, and the robot
picks + places it — a full perceive→ground→act loop. Needs `pip install transformers`
(weights auto-download, ~600 MB, cached); the model runs in a subprocess
(`r2d3_sim/perception.py`) since `transformers` crashes the Isaac kit process in-line.
Writes `tests/captures/clear_island.gif` + `clear_island_detect.png`.

## Switch the arm / end-effector

Examples `01`–`07` take **`--ee dexterous|gripper`** to switch between the 5-finger
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
