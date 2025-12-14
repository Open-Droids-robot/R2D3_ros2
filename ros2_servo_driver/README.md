## **1.Project Introduction**
ros2_servo_driver It is a ros2 head servo package for a double-arm composite robot.The functions implemented are:

1. Serial port control head servo
1. Get the angle of the servo in real time

There are also demo test cases for various functions

## **2. File Structure**
```bash
├── servo_driver            # Servo driver function package
│   ├── CMakeLists.txt
│   ├── include
│   ├── launch
│   ├── scripts             # Store udev scripts
│   ├── package.xml
│   └── src
├── servo_example           # Sample function package
│   ├── CMakeLists.txt
│   ├── include
│   ├── package.xml
│   └── src
└── servo_interfaces        # Servo message function package
    ├── CMakeLists.txt
    ├── include
    ├── msg
    ├── package.xml
    └── src
```

## **3. Compilation Method**
```bash
cd rm_dual_arm_robot_ros2/ros2_servo_driver/servo_driver/scripts
# Execute the udev script and restart the device after execution
sudo bash servo_udev.sh
# Enter the workspace directory
cd rm_dual_arm_robot_ros2/     
colcon build --packages-select servo_interfaces servo_driver servo_example 
```

## **4. Run Commands**

- 1.Start the overall ros2 function package launch

```bash
cd rm_dual_arm_robot_ros2/ 
source install/setup.bash                                # Reload the workspace environment variables
ros2 launch servo_driver servo_start.launch.py           # Start the servo driver
```

- 2.Use cases for starting function package function：

```bash
ros2 run servo_example send_control_cmd_demo              # Control the servo
ros2 run servo_example get_servo_angle_demo               # Get the servo angle in real time
```

