# Using R2D3 as a platform

The `R2D3` SDK ([api.md](api.md)) is a clean control + sensing surface. Three
ready-made interfaces sit on top of it in `isaac_sim/r2d3_sim/envs/`, and the
`setup` hook lets you build arbitrary task scenes. Everything below is in-process
Python — no ROS required.

## Reinforcement learning — `R2D3Env` (Gymnasium)

`isaac_sim/r2d3_sim/envs/rl_env.py` wraps the robot as a standard `gymnasium.Env`,
so it drops into SB3 / CleanRL / rllib.

```python
from isaac_sim.r2d3_sim.envs.rl_env import R2D3Env

def reward(obs):      # obs is an R2D3 Observation
    return -float(np.linalg.norm(obs.ee_pose_left[0] - target))

env = R2D3Env(control="ee_delta", reward_fn=reward, done_fn=lambda o: ...)
obs, _ = env.reset()
obs, r, terminated, truncated, info = env.step(env.action_space.sample())
```

- **Observation**: head RGB + left-arm joints + lift + wrist wrench (a `Dict` space).
- **Action**: `ee_delta` (Δx,Δy,Δz + gripper, default), `joint_delta`, or `joint_abs`.
- **Reward / termination**: your callables over the `Observation`.
- Needs `pip install gymnasium`. Example: `examples/04_rl_env.py`.

## Vision / VLM policies — `PerceptionLoop`

`envs/vlm_loop.py` runs a perceive → decide → act loop. Your `policy(rgb, obs)`
returns a high-level action dict (`arm_pose`, `arm_joints`, `gripper`, `head`,
`lift`, `base_pose`) which `apply_action` maps onto the SDK setters.

```python
from isaac_sim.r2d3_sim.envs.vlm_loop import PerceptionLoop

def policy(rgb, obs):
    # call your VLM on `rgb` (HxWx3); return an action
    return {"arm_pose": {"side": "left", "position": grasp_xyz}}

PerceptionLoop(sim, policy, camera="head").run(n_iters=10)
```

Example: `examples/05_vlm_loop.py` (stub policy). Swap in a real model — the head
image and full observation are the inputs.

## Teleoperation — `TeleopServer`

`envs/teleop.py` latches the latest targets and applies them each tick. Feed it
from a keyboard, gamepad, VR controller, leader arm, or socket.

```python
from isaac_sim.r2d3_sim.envs.teleop import TeleopServer
tele = TeleopServer(sim)                       # use_ros=True to take /r2d3/sim/cmd/*
tele.submit(left_ee=(xyz, sim.top_down_quat), gripper_left=0.0)
tele.spin(40)                                  # apply + step
```

Example: `examples/03_teleop.py`.

## Custom task scenes — the `setup` hook

Add objects (tables, cubes, props) before physics initialises by passing a
`setup(world)` callable; create rigid/fixed bodies with
`isaacsim.core.api.objects`:

```python
def build(world):
    from isaacsim.core.api.objects import DynamicCuboid
    DynamicCuboid(prim_path="/World/cube", position=[0.5,-0.2,0.46], scale=[0.04]*3, mass=0.03)

sim = R2D3(setup=build)
sim.reset()
```

See `examples/07_grasp_cube.py` for a full grasp-and-lift task (IK + weld + lift).

## Tips

- **Grasping small objects** is unreliable with friction alone; the example
  attaches the object to the hand with a fixed joint at contact (a deliberate V1
  simplification). Real contact grasping needs the dynamics work tracked for V2.
- **Driving** is kinematic in V1 (`set_base_pose` + visually spin wheels with
  `set_joint_targets`); the chassis is otherwise pinned. Wheel-physics driving is V2.
- **Both arms**: joint control works for both (`set_arm_joints("right", q)`); IK
  (`set_arm_pose`) is left-only until a right-arm Lula description is added.
