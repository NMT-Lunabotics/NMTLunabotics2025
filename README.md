# NMTLunabotics2025
The main repository for the New Mexico Tech Lunabotics 2025 competition Team

# INSTALLATION

Install docker  
<https://docs.docker.com/engine/install/ubuntu/>  

To use docker without sudo, run the following commands  
```
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Ensure you have an SSH key set up with Github  
<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>  

Clone this repository with SSH and cd into it  
```
git clone git@github.com:NMT-Lunabotics/NMTLunabotics2025.git
cd NMTLunabotics2025
```

# USAGE

`start_docker.sh`: Start docker container and enter a bash terminal  
`rtabmap.sh`: Start docker container and run rtabmap with a realsense camera  
`teleop.sh`: Start docker container and run controller teleop  
`rviz.sh`: Run rviz in the container  
`docker_term.sh`: Open a bash terminal in the container  

# Helpful Docs
<https://github.com/ros2/teleop_twist_joy/tree/humble/>
<https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md>

# Launch Files

To set arguments, append to the end of the launch command  
`ros2 launch package file_launch.py arg1:=value1 arg2:=value2`

Parameters are set in the same way. To change parameters permanently, change the value in the yaml config file or in the launch file.
Use `ros2 param list` to view parameters for running nodes.

## nukes_launch.py
Package: robot_uprising
`ros2 launch robot_uprising nukes_launch.py`
This launches all operations on the robot: motion, slam, and navigation. In event of existential crisis, run `sudo systemctl stop sentient_bridge.service`. Occasionally, GOLIATH will remove user sudo permissions to prevent this command from running. If the previous command fails, remove GOLIATH's power source as soon as possible.
Arguments:
 - launch_motors: true. Launches motor_control_launch.py
 - launch_mapping: true. Launches rtabmap_launch.py
 - launch_nav: true. Launches nav_launch.py

## teleop_launch.py
Package: teleop
Converts a usb joystick (video game controller) into cmd_vel commands. Parameters are stored in the launch file. See [joy](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy) and [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy/tree/humble/) documentation.

## motor_control_launch.py
Package: motor_control
`ros2 launch motor_control motor_control_launch.py`
Sends serial commands to the arduino to control the left and right motors. Parameters for GOLIATH are stored in [motor_control_params.yaml](ros2_ws/src/motor_control/config/motor_control_params.yaml).

## rtabmap_launch.py
Package: slam_config
`ros2 launch slam_config rtabmap_launch.py`
Launches rtabmap with a single realsense camera. Provides 2d and 3d map. Parameters are stored in the launch file. See [rtabmap](https://github.com/introlab/rtabmap_ros/tree/ros2) and [realsense_ros](https://github.com/IntelRealSense/realsense-ros) documentation for details.

## slam_launch.py
Package: slam_config
`ros2 launch slam_config slam_launch.py`
Launches slam_toolbox with the rplidar s1 camera. It uses ros2_laser_scan_matcher to generate odometry from the laserscan. Provides a 2d map. See [nav2 documentation](https://docs.nav2.org/tutorials/index.html) for information on how to use slam_toolbox and the rest of the ros2 navigation stack. Parameters are stored in the launch file.

## nav_launch.py
Package: navigation
`ros2 launch navigation nav_launch.py`
Path planning with a costmap provided by rtabmap or slam_toolbox. Uses parameters from [nav2_params.yaml](ros2_ws/src/navigation/config/nav2_params.yaml) See nav2 tutorial for information.

