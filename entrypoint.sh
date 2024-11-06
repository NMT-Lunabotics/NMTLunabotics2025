#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/humble/setup.bash
source /home/luna/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo chmod 666 /dev/ttyACM0

# Launch the ROS nodes, including the RealSense D455 camera
exec "$@"

