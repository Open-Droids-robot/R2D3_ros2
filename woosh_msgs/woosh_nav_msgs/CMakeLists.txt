cmake_minimum_required(VERSION 3.8)
project(woosh_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(woosh_common_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Path.msg"
  "msg/PlanPathOptimal.msg"
  "msg/PlanPath.msg"
  "msg/ArrType.msg"
  "msg/Mode.msg"
  "msg/ModeSetting.msg"
  DEPENDENCIES woosh_common_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
