# Woosh Robot ROS Interface - Developer Notes

⚠️ **Disclaimer**:  
This document is **not** the official WooshRobot manual.  
It is a set of **developer notes and usage examples** based on the Woosh Robot ROS Interface.  
The original documentation is © 2025 WooshRobot. For complete and authoritative reference, please consult the official WooshRobot documentation.

---

## Introduction
These notes summarize the available ROS 2 interfaces for the Woosh Robot, including services, topics, and actions.  
Examples are provided to demonstrate practical usage.  
Full parameter descriptions are omitted — refer to the official docs.

---

## Development and Operating Environment
The Woosh Robot agent can be deployed on:
- Ubuntu 24.04 + ROS 2 Jazzy
- Ubuntu 22.04 + ROS 2 Humble
- Ubuntu 20.04 + ROS 2 Foxy (AMD64 / ARM64)

---

## Deploy and Run

1. Get the appropriate installation package:  
   `ros-xxx-woosh-robot-agent_xxx_xxx.run`

2. Run the agent with namespace `/woosh_robot` (recommended):  

```bash
chmod +x ros-xxx-woosh-robot-agent_xxx_xxx.run
./ros-xxx-woosh-robot-agent_xxx_xxx.run

ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="172.20.8.74"
```

---

## Interfaces Overview

The robot exposes three types of ROS interfaces:  
- **Services** (request-response)  
- **Topics** (publish-subscribe)  
- **Actions** (long-running tasks with feedback)  

---

## Example Interfaces

### 1. Robot Information
Get general info about the robot.

```bash
ros2 service call /woosh_robot/robot/RobotInfo woosh_robot_msgs/srv/RobotInfo
```

---

### 2. Robot State
Retrieve current state (e.g., Idle, Charging, Task, Warning).

```bash
ros2 service call /woosh_robot/robot/RobotState woosh_robot_msgs/srv/RobotState
ros2 topic echo /woosh_robot/robot/RobotState
```

---

### 3. Robot Battery
Get battery charge status, health, and cycle count.

```bash
ros2 service call /woosh_robot/robot/Battery woosh_robot_msgs/srv/Battery
ros2 topic echo /woosh_robot/robot/Battery
```

---

### 4. Navigation Path
Retrieve or modify the robot’s navigation path.

```bash
ros2 service call /woosh_robot/robot/NavPath woosh_robot_msgs/srv/NavPath
ros2 service call /woosh_robot/robot/ChangeNavPath woosh_robot_msgs/srv/ChangeNavPath "{arg:{paths:{...}}}"
```

---

### 5. Task Execution (Action)
Execute a task such as navigation or transport.

```bash
ros2 action send_goal /woosh_robot/robot/ExecTask woosh_robot_msgs/action/ExecTask "{arg:{type:{value: 1}, mark_no: A2}}" --feedback
```

---

### 6. Remote Control (Twist)
Send velocity commands.

```bash
ros2 service call /woosh_robot/robot/Twist woosh_robot_msgs/srv/Twist "{arg:{linear: 0.2, angular: 0.785}}"
```

---

### 7. Voice Broadcast
Make the robot speak.

```bash
ros2 service call /woosh_robot/robot/Speak woosh_robot_msgs/srv/Speak "{arg:{text: 'Hello World'}}"
```

---

## Additional Features

- **Robot Settings**: Configure identity, server, auto-charge, auto-park, sound, etc.  
- **Map Management**: Retrieve scene/map data.  
- **Task Management**: Request or query predefined/call tasks.  
- **Step Control**: Move forward, rotate, or lateral moves.  
- **Lift Control**: Control lifting mechanisms.  
- **Basic Navigation**: Send target poses.  

---

## Notes for Developers
- Always run under namespace `/woosh_robot`.  
- Some interfaces support both **service** and **topic** usage.  
- Control mode and work mode must be switched before executing tasks.  
- Navigation and task actions provide **feedback** for progress tracking.  

---

© 2025 WooshRobot (original documentation).  
This file is a **developer-friendly summary** created for GitHub-safe usage.


---

# ros2_agv_robot Project Notes

## **1. Project Introduction**
`ros2_agv_robot` is a ROS2 chassis package for embossed two-arm robots.  
It provides demos and test cases for various chassis-related functions.

### Functions implemented:
1. Request robot information  
2. Subscribe to robot information  
3. Subscribe to the robot status  
4. Subscribe to mode information  
5. Subscribe to position speed  
6. Subscribe to battery information  
7. Subscribe to scene information  
8. Subscribe to task progress  
9. Subscribe to device status  
10. Subscribe to run status  
11. Task execution  
12. Wait for the task execution result  
13. Task execution request  
14. Callbacks in task execution process  
15. Perform predefined tasks  
16. Stepping process callback  
17. Stepping execution request  
18. Wait for stepping execution results  

---

## **2. File Structure**

```bash
├── ros2_agv_robot           # 底盘功能包
│   ├── CMakeLists.txt       # 编译规则文件
│   ├── lib                  # ros2接口安装包
│   ├── package.xml          # 定义功能包属性文件
│   └── src                  # 测试代码目录
```

---

## **3. Compilation Method**

1. Install the chassis ROS2 interface installation package:

```bash
cd ~/ros2_agv_robot/lib
sudo ./ros-foxy-woosh-robot-agent_0.0.1-0focal_arm64.run
```

2. Compile and test demo:

```bash
cd ros2_ws/     # 进入工作空间目录
colcon build --packages-select ros2_agv_robot
```

---

## **4. Run Commands**

1. Start the chassis connection node:

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
# or
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot
```

2. Run test/demo functions:

```bash
ros2 run ros2_agv_robot monitor
# Execute the monitor node to obtain robot information

ros2 run ros2_agv_robot executeask --ros-args -p mark_no:=7613B5D
# When a certain storage bit is reached (7613B5D), 
# you need to create a map in advance, initialize the robot, and then execute.

ros2 run ros2_agv_robot exec_pre_task --ros-args -p id:=1735291765
# Execute the task (1735291765), requires creating the map and initializing the robot first.

ros2 run ros2_agv_robot stepctrl
# Step control
```

---


© 2025 WooshRobot (original documentation).  
This file is a **developer-friendly summary** created for GitHub-safe usage.
for official documentation : https://seafile.wsrobotics.com/d/9fee6b1fda3b4403919c/?p=%2F%E8%BD%AF%E4%BB%B6%E4%BA%8C%E6%AC%A1%E5%BC%80%E5%8F%91%E6%8E%A5%E5%8F%A3%2F%E5%BA%94%E7%94%A8%E5%B1%82ROS2%E6%8E%A5%E5%8F%A3%2Fv0.0.1&mode=list
