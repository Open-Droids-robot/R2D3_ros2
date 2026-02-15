cmake_minimum_required(VERSION 3.8)
project(r2d3_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
