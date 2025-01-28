#!/bin/bash
set -e
sudo chmod 666 /dev/video0
. /opt/ros/humble/setup.sh
. /home/luna/ros2_ws/install/setup.sh
. /home/luna/.bashrc
exec "$@"

