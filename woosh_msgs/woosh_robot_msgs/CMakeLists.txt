cmake_minimum_required(VERSION 3.8)
project(woosh_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(woosh_common_msgs REQUIRED)
find_package(woosh_action_msgs REQUIRED)
find_package(woosh_task_msgs REQUIRED)
find_package(woosh_nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Enum types
  "msg/State.msg"
  "msg/ControlMode.msg"
  "msg/WorkMode.msg"
  "msg/BatteryChargeState.msg"
  "msg/FootPrint.msg"
  "msg/Type.msg"
  "msg/NetworkWiFiMode.msg"
  # Simple composite types
  "msg/GeneralModelData.msg"
  "msg/GeneralVersion.msg"
  "msg/Identity.msg"
  "msg/Server.msg"
  "msg/Power.msg"
  "msg/Sound.msg"
  "msg/SettingAllow.msg"
  "msg/AutoCharge.msg"
  "msg/AutoPark.msg"
  "msg/GoodsCheck.msg"
  "msg/Twist.msg"
  "msg/ExecPreTaskArg.msg"
  # Composite types
  "msg/Mode.msg"
  "msg/Battery.msg"
  "msg/PoseSpeed.msg"
  "msg/Scene.msg"
  "msg/TaskProcAction.msg"
  "msg/TaskProc.msg"
  "msg/DeviceStateHardwareBit.msg"
  "msg/DeviceStateSoftwareBit.msg"
  "msg/DeviceState.msg"
  "msg/OperationStateNavBit.msg"
  "msg/OperationStateRobotBit.msg"
  "msg/OperationState.msg"
  "msg/RobotState.msg"
  "msg/General.msg"
  "msg/Model.msg"
  "msg/NavPath.msg"
  "msg/PlanPath.msg"
  "msg/NetworkWiFi.msg"
  "msg/Network.msg"
  "msg/Setting.msg"
  "msg/ScannerData.msg"
  "msg/ExecTask.msg"
  "msg/RobotInfo.msg"
  # Services
  "srv/RobotInfo.srv"
  "srv/ExecPreTask.srv"
  # Actions
  "action/ExecTask.action"
  DEPENDENCIES woosh_common_msgs woosh_action_msgs woosh_task_msgs woosh_nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
