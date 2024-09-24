# NMTLunabotics2025
The main repository for the New Mexico Tech Lunabotics 2025 competition Team

# Installation

This system has been tested on Ros 2 humble with ubuntu 22. Teleop node works on Ros 2 Foxy with Ubuntu 20.

Robot dependencies
TODO: Make sure this list is complete
```
sudo apt install -y ros-humble-joy
sudo apt install -y ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-realsense2-camera
sudo apt install -y ros-humble-librealsense2
sudo apt install -y humble-rmw-cyclonedds-cpp
sudo apt install -y ros2-humble-usb-cam
sudo apt install -y ros-humble-image-transport-plugins
```

Remote dependencies
```
sudo apt install -y ros-humble-joy
sudo apt install -y ros-humble-teleop-twist-joy
sudo apt install -y humble-rmw-cyclonedds-cpp
```

# Usage

## Robot

Run slam toolbox with rplidar
`ros2 launch slam_config slam_launch.py`

## Remote

Run teleop node
`ros2 launch teleop teleop_launch.py`
