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

See [realsense-ros2-docker](https://github.com/2b-t/realsense-ros2-docker/tree/b8ceee5b17634996cca1bd7c50b12fc588c581c8?tab=readme-ov-file#2-launching) to setup docker environment  


# USAGE

Run the docker image  
`$ docker compose -f docker-compose-gui.yml up`

If gui is not needed
`$ docker compose -f docker-compose.yml up`

