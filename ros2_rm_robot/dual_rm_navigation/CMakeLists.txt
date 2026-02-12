cmake_minimum_required(VERSION 3.8)
project(dual_rm_navigation)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY
  config
  launch
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
