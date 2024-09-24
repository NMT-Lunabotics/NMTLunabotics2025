# NMTLunabotics2025
The main repository for the New Mexico Tech Lunabotics 2025 competition Team

# Installation

Robot dependencies
TODO: Make sure this list is complete
```
sudo apt install -y ros-humble-joy
sudo apt install -y ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-realsense2-camera
sudo apt install -y ros-humble-librealsense2
ros-humble-rmw-cyclonedds-cpp
ros2-humble-usb-cam
ros-humble-image-transport-plugins
```

Remote dependencies
```
sudo apt install -y ros-humble-joy
sudo apt install -y ros-humble-teleop-twist-joy
```

# Usage

## Robot

Run slam toolbox with rplidar
`ros2 launch slam_config slam_launch.py`

## Remote

Run teleop node
`ros2 launch teleop teleop_launch.py`
