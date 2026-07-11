cmake_minimum_required(VERSION 3.8)
project(woosh_common_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2D.msg"
  "msg/Twist.msg"
  "msg/Point.msg"
  "msg/FileData.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
