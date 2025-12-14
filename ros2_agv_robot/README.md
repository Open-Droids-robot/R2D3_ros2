## **1. Project Introduction**
ros2_agv_robot is a ROS2 chassis package for embodied dual-arm robots. The implemented functions include:

1. Request robot information
2. Subscribe to robot information
3. Subscribe to robot status
4. Subscribe to mode information
5. Subscribe to pose velocity
6. Subscribe to battery information
7. Subscribe to scene information
8. Subscribe to task progress
9. Subscribe to device status
10. Subscribe to running status
11. Task execution
12. Wait for task execution results
13. Task execution request
14. Task execution process callback
15. Execute predefined tasks
16. Step process callback
17. Step execution request
18. Wait for step execution results

There are also demo test cases for various functions



## **2. File Structure**
```bash
├── ros2_agv_robot           # Chassis function package
│   ├── CMakeLists.txt       # Compilation rule file
│   ├── lib                  # ros2 interface installation package
│   ├── package.xml          # Package property definition file
│   └── src                  # Test code directory

## **3. Compilation Method**
1. Install chassis ros2 interface installation package
cd ~/ros2_agv_robot/lib
sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run
2. Compile test demo
```bash
cd ros2_ws/     # Enter workspace directory
colcon build --packages-select ros2_agv_robot
```

## **4. Run Commands**
- 1. Start chassis connection node

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
or
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot
```

- 2. Use cases for starting function package functions:

```bash
ros2 run ros2_agv_robot monitor  				          	        # Execute monitor node to get robot information
ros2 run ros2_agv_robot exectask --ros-args -p mark_no:=7613B5D     # Reach a storage position (7613B5D), need to create map in advance, robot initialization, create storage position and then execute
ros2 run ros2_agv_robot exec_pre_task --ros-args -p id:=1735291765  # Execute task (1735291765), create map in advance, robot initialization, create task and then execute
ros2 run ros2_agv_robot stepctrl                                    # Step control
```