cmake_minimum_required(VERSION 3.8)
project(woosh_action_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Type.msg"
  "msg/State.msg"
  "msg/Order.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
