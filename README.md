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
`docker_term.sh`: Open another terminal in the same container  
`rtabmap.sh`: Start docker container and run rtabmap with a realsense camera  
`teleop.sh`: Start docker container and run controller teleop

# Helpful Docs
<https://github.com/ros2/teleop_twist_joy/tree/humble/>
<https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md>
