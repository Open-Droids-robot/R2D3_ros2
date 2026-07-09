cmake_minimum_required(VERSION 3.5)
project(ros2_total_demo)

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
find_package(rclpy REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(servo_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPose.msg"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${PROJECT_SOURCE_DIR}  
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

set(
  SRC
  ${PROJECT_SOURCE_DIR}/src/total_demo_node.cpp
  ${PROJECT_SOURCE_DIR}/src/dual_arm.cpp
)

add_executable(total_demo_node ${SRC})
target_include_directories(total_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  total_demo_node
  "rclcpp"
  "std_msgs"
  "rm_ros_interfaces"
  "servo_interfaces"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "realsense2"
)

target_link_libraries(total_demo_node
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

install(TARGETS total_demo_node
DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME}) 

install(PROGRAMS 
  scripts/camera_0.py 
  scripts/camera_1.py
  scripts/realsense_camera_0.py 
  scripts/realsense_camera_1.py 
  scripts/realsense_camera_2.py 
  scripts/detect_object.py
  scripts/catch2object_aoyi_hand.py
  scripts/catch2object_gripper.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY weight/
  DESTINATION lib/${PROJECT_NAME}/weight
)

install(DIRECTORY launch msg scripts DESTINATION share/${PROJECT_NAME})

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
