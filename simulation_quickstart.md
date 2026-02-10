# Simulation Quickstart (ROS 2 Jazzy)

This workspace has been updated to support **Gazebo Harmonic (Gz Sim)**, the default simulator for ROS 2 Jazzy.

> **Note**: The legacy `gazebo_ros` (Gazebo Classic) launch files are still present but will not work on Jazzy. Use the `_gz_sim` variants below.

---

### 1. Launch Emulation with Gz Sim

Choose your arm variant (65B or 75B):

```bash
# For 65B (6-DOF) arm
ros2 launch dual_rm_gazebo dual_rm_65b_gz_sim.launch.py

# For 75B (7-DOF) arm
ros2 launch dual_rm_gazebo dual_rm_75b_gz_sim.launch.py
```

This will:
- Launch Gz Sim (Harmonic)
- Spawn the robot model
- Load `ros_gz_bridge` for `/clock` (time sync)
- Load `ros2_control` controllers (`joint_state_broadcaster`, `left/right_arm_controller`, `platform_controller`)

---

### 2. Launch MoveIt 2 (Motion Planning)

In a new terminal:

```bash
# Source workspace
source install/setup.bash

# For 65B arm
ros2 launch dual_rm_65b_moveit_config demo.launch.py

# For 75B arm
ros2 launch dual_rm_75b_moveit_config demo.launch.py
```

---

### 3. Run MoveIt Demo

In a third terminal:

```bash
# Source workspace
source install/setup.bash

# For 65B arm
ros2 launch dual_rm_moveit_demo rm_65_moveit2_fk.launch.py

# For 75B arm
ros2 launch dual_rm_moveit_demo rm_75_moveit2_fk.launch.py
```

---

### Troubleshooting

If controllers fail to load or simulation time is not syncing:
1. Ensure `ros-jazzy-ros-gz-sim` and `ros-jazzy-gz-ros2-control` are installed.
2. Check that the `/clock` bridge is running (it is included in the launch file).
3. Verify `use_sim_time` is set to `true` for all nodes (handled by launch files).