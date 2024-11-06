#!/bin/bash

docker build -t ros2_luna .
docker run -it --rm \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --network=host \
  --privileged \
  --name \
  luna \
  ros2_luna \
  ros2 launch teleop teleop_launch.py

