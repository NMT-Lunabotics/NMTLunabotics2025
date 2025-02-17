#!/bin/bash
set -e
if [ -e /dev/video0 ]; then
    sudo chmod 666 /dev/video0
fi
. /opt/ros/humble/setup.sh
. /home/luna/ros2_ws/install/setup.sh
. /home/luna/.bashrc
exec "$@"

