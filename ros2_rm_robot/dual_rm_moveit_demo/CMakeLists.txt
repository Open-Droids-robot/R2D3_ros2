cmake_minimum_required(VERSION 3.16)
project(dual_rm_moveit_demo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(moveit_visual_tools REQUIRED)

add_executable(moveit2_fk_demo src/moveit2_fk_demo.cpp)
add_executable(moveit2_ik_demo src/moveit2_ik_demo.cpp)

target_include_directories(moveit2_fk_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  moveit2_fk_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

ament_target_dependencies(
  moveit2_ik_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)


install(TARGETS
  moveit2_fk_demo
  moveit2_ik_demo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
