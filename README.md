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
```
Usage: ./start_docker.sh [--start (-s) | --motor_ctrl (-m) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |
           --mapping (-M) | --nav (-n) | --save-map (-S) | --copy-map (-C) |
           --command (-c) <command> | --build (-b) | --stop (-x) |
           --restart (-r)] [--display (-d)] [--quiet (-q)] [--ros-domain-id (-i) <id>] [--help (-h)]
This script is used to start and manage a Docker container for ROS2
If no action is specified, the script will open an interactive bash terminal in the container.
Actions (pick ONE):
  --start (-s)                Start all processes on the robot
  --motor_ctrl (-m)           Run motor control
  --teleop (-t)               Run joystick control using teleop.launch
  --usb-cam (-u)              Run usb camera node using usb_cam.launch
  --video-stream (-v)         View the video stream using view_camera.launch
  --mapping (-M)              Run rtabmap_launch.py from slam_config
  --nav (-n)                  Run nav_launch.py from navigation
  --save-map (-S)             Save map using nav2_map_server map_saver_cli
  --copy-map (-C)             Copy map files from Docker to ./maps directory
  --command (-c) <command>    Pass a command to be run in the container
Options:
  --display (-d)              Enable display support (forward X11 display)
  --build (-b)                Build the Docker container (will stop the running container if any)
  --stop (-x)                 Stop the running Docker container
  --restart (-r)              Restart the Docker container if it is running
  --quiet (-q)                Suppress output
  --ros-domain-id (-i) <id>   Set the ROS_DOMAIN_ID (default: 42)
  --help (-h)                 Show this help message
```

# Ethernet
To ssh into the jetson via ethernet, set static ip adresses for the ethernet port on both computers. Replace eth0 with the name of your ethernet device.
```
sudo ip addr flush dev eth0
sudo ip addr add dev eth0 192.168.50.xxx/24
```

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

