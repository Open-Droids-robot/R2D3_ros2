cmake_minimum_required(VERSION 3.8)
project(woosh_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(woosh_common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/ControlAction.msg"
  "msg/Feedback.msg"
  "msg/StepControlStepMode.msg"
  "msg/StepControlStep.msg"
  "msg/StepControl.msg"
  "msg/LiftControlExecuteMode.msg"
  "msg/LiftControl.msg"
  "msg/LiftControl3ExecuteMode.msg"
  "msg/LiftControl3.msg"
  "msg/MoveBaseExecutionMode.msg"
  "msg/MoveBase.msg"
  "action/StepControl.action"
  "action/LiftControl.action"
  "action/LiftControl3.action"
  "action/MoveBase.action"
  DEPENDENCIES woosh_common_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
