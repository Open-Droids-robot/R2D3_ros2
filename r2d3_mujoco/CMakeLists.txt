cmake_minimum_required(VERSION 3.8)
project(r2d3_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ensure_mjcf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
