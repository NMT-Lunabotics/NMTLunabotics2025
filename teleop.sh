#!/bin/bash

docker build -t ros2_luna .
docker run -it --rm \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --network=host \
  --name \
  luna \
  ros2_luna \
  bash -c "source ~/.bashrc && ros2 launch teleop teleop_launch.py"

