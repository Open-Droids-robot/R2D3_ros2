# The ROS bridge (two-process path)

Most users want the in-process [SDK](api.md). Use the ROS path when you need the
custom `rm_ros_interfaces` message surface that the real R2D3 model stack /
MoveIt2 / the eval harness speak.

## Why two processes

Isaac Sim 6.0 ships a **runtime-only** bundled ROS 2 Humble for Python 3.12
(rclpy + standard message libs, but no `share/` or CMake configs), so custom
interfaces can't be built against it. The split:

```
model stack (rm_ros_interfaces, Py 3.11, ros_humble env)
        │  Movej / Movejp / Gripperset / Liftheight  (rm_driver topics)
        ▼
r2d3_humble_bridge (Py 3.11, ros_humble env)
        │  std_msgs / sensor_msgs on the /r2d3/sim/* namespace
        ▼
Isaac sim_adapter (Py 3.12, isaac env, bundled rclpy)
        │  articulation targets / joint state
        ▼
PhysX articulation
```

Isaac speaks **only standard messages**; the bridge translates the custom types.
Cost: one extra DDS hop (~1 ms on loopback).

## Running it

```bash
# process 1 — Isaac, publishing/subscribing on /r2d3/sim/*
scripts/isaacsim_ros2.sh isaac_sim/r2d3_sim/bring_up.py        # --headless / --no-ros

# process 2 — the translator (in the ros_humble env)
ros2 run r2d3_humble_bridge bridge
```

`bring_up.py` boots the app, loads the scene, wires the OmniGraph publishers
(`/clock`, `/tf`, the D435 cameras), and runs `sim_adapter` (the rclpy node).

## Topic surface (`/r2d3/sim/*`)

Defined in `isaac_sim/r2d3_sim/sim_topics.py` (the Isaac↔bridge contract).

| direction | topic | type |
|---|---|---|
| state out | `/r2d3/sim/joint_states` | `sensor_msgs/JointState` (all DOFs) |
| state out | `/r2d3/sim/lift_state` | `std_msgs/Float64` |
| state out | `/r2d3/sim/ready` | `std_msgs/Bool` (latched) |
| cmd in | `/r2d3/sim/cmd/{left,right}_arm` | `sensor_msgs/JointState` (targets) |
| cmd in | `/r2d3/sim/cmd/{left,right}_finger` | `std_msgs/Float64` (drive, m) |
| cmd in | `/r2d3/sim/cmd/lift` | `std_msgs/Float64` (m) |
| cameras | `/r2d3/sim/head/*`, `/r2d3/sim/{l,r}_wrist/*` | `Image` / `CameraInfo` |

The bridge re-aggregates state into `Armstate` / `Liftstate` and the full
`r2d3_model_interfaces/Observation` on `/r2d3/observations`.

## SDK + ROS together

`R2D3(enable_ros=True)` runs `sim_adapter` inside the facade process — you get the
in-process numpy API *and* the `/r2d3/sim/*` topics from one script. `step()`
flushes incoming ROS commands into the articulation each tick. `TeleopServer(sim,
use_ros=True)` drives the robot straight from `/r2d3/sim/cmd/*`.

See [architecture.md](architecture.md) for the full design and the dynamics /
gripper-mimic decisions.
