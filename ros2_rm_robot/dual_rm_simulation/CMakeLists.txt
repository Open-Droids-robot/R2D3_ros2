cmake_minimum_required(VERSION 3.8)
project(dual_rm_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stereo_concat.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
