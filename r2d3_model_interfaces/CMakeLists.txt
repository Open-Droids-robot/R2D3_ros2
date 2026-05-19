cmake_minimum_required(VERSION 3.8)
project(r2d3_model_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Observation.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs rm_ros_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
