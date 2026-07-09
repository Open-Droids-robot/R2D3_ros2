## **1. Project Introduction**
ros2_total_demo is an overall test of the two-arm composite robot package. Including camera, lift, double arms, head servo

## **2. File Structure**
```bash
├── CMakeLists.txt
├── include
│   └── ros2_total_demo
├── launch
│   └── total_demo.launch.py
├── package.xml
├── scripts
│   ├── camera_0.py
│   ├── camera_1.py
│   ├── realsense_camera_0.py
│   ├── realsense_camera_1.py
│   └── realsense_camera_2.py
└── src
    ├── camera.cpp
    ├── dual_arm.cpp
    └── total_demo_node.cpp
```

## **3. Compilation Method**
```bash
cd rmc_aida_l_ros2/     # Enter the workspace directory
colcon build --packages-select ros2_total_demo
```

## **4. Run Commands**

- 1.Launch the launch of the overall ros2 function package

```bash
cd rm_dual_arm_robot_ros2/ 
source install/setup.bash                                  # Reload the workspace environment variables
ros2 launch ros2_total_demo total_demo.launch.py           # Two arms, head servo drive, camera
```

- 2.Use cases for starting function package function：

```bash
ros2 run ros2_total_demo total_demo_node                   # Start the overall demo
```

