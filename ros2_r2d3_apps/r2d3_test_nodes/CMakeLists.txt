cmake_minimum_required(VERSION 3.8)
project(r2d3_test_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(include)

add_executable(test_agv_motion src/test_agv_motion.cpp)
ament_target_dependencies(test_agv_motion
  rclcpp geometry_msgs
)

add_executable(test_arm_motion src/test_arm_motion.cpp)
ament_target_dependencies(test_arm_motion
  rclcpp rclcpp_action trajectory_msgs control_msgs
)

install(TARGETS
  test_agv_motion
  test_arm_motion
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
