#!/bin/bash

# Build the container
docker build -t r2d3-humble .

# Run the container with proper ROS 2 DDS communication
docker run -it --rm \
    --user=ros \
    --name r2d3-humble \
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
    r2d3-humble