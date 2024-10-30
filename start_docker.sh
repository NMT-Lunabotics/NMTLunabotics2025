#!/bin/bash

docker run -it --rm \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --network=host \
  --privileged \
  --name \
  luna \
  ros2_luna \
  bash

