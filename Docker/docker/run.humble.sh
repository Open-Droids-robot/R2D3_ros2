#!/bin/bash

# Build the container for ROS2 Humble
docker build -f docker/Dockerfile.humble -t ros2-humble-r2d3 .

# Run the container with proper ROS 2 DDS communication
docker run -it --rm \
    --user=ros \
    --name ros2-humble-r2d3-container \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    --net=host \
    --ipc=host \
    --privileged \
    --device=/dev/dri:/dev/dri \
    --security-opt apparmor=unconfined \
    -e ROS_DOMAIN_ID=1 \
    -v /dev/shm:/dev/shm:rw \
    -v /run/user/$(id -u)/pulse:/run/user/1000/pulse \
    -e XDG_RUNTIME_DIR=/run/user/1000 \
    -e PULSE_RUNTIME_PATH=/run/user/1000/pulse \
    -e ROS_DISTRO=humble \
    ros2-humble-r2d3
