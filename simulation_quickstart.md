## Simulation Quickstart

Once you have the Docker environment running, follow the steps below to bring up the dual RM simulation stack.

### Choose Your Robot
- `65` for the 6-DoF arm  
- `75` for the 7-DoF arm

### 1. Launch Gazebo
```bash
ros2 launch dual_rm_65b_gazebo dual_rm_65b_gazebo.launch.py
# or
ros2 launch dual_rm_75b_gazebo dual_rm_75b_gazebo.launch.py
```

### 2. Launch MoveIt
```bash
ros2 launch dual_rm_65b_moveit_config demo.launch.py
# or
ros2 launch dual_rm_75b_moveit_config demo.launch.py
```

### 3. Run MoveIt Example
```bash
ros2 launch dual_rm_moveit_demo rm_65_moveit2_fk.launch.py
# or
ros2 launch dual_rm_moveit_demo rm_75_moveit2_fk.launch.py
```

---

This workflow covers the essentials for simulation. Feel free to extend the setup with Nav2 for navigation or integrate cameras and LiDAR for your own VSLAM pipeline—this is a bare-bones structure so you can develop without limits.