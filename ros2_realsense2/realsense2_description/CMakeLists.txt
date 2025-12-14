cmake_minimum_required(VERSION 3.5)
#Compiled with old python rules to remove warnings
if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD) 
endif()
project(realsense2_description)

find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
