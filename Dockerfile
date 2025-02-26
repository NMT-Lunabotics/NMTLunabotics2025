FROM ros:humble
ENV DISPLAY=0

RUN apt-get update && apt-get install -y sudo

# Create a non-root user named 'luna'
ARG USER=luna
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN usermod -aG dialout $USER

USER $USER
WORKDIR /home/$USER

# Install system packages
USER root
RUN apt-get install -y python3
RUN apt-get install -y python3-pip

# Install python packages
RUN pip3 install pyserial

# Install ros packages
RUN apt-get -y install ros-humble-teleop-twist-joy
RUN apt-get -y install ros-humble-joy
RUN apt-get -y install ros-humble-navigation2
RUN apt-get -y install ros-humble-nav2-bringup
RUN apt-get -y install ros-humble-realsense2-*
RUN apt-get -y install ros-humble-rviz2
RUN apt-get -y install ros-humble-rtabmap-ros
RUN apt-get -y install ros-humble-rmw-cyclonedds-cpp
RUN apt-get -y install ros-humble-usb-cam
RUN apt-get -y install ros-humble-image-view
RUN apt-get -y install ros-humble-image-transport-plugins
RUN apt-get -y install python3-pydantic
RUN apt-get -y install v4l-utils
RUN apt-get -y install ros-humble-rosidl-generator-py


# Copy in the ros workspace
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws
COPY --chown=$USER:$USER rviz2 /home/$USER/.rviz2

# Compile the ros workspace
USER $USER
WORKDIR /home/$USER/ros2_ws
# RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build --packages-skip slam_config navigation'
RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build'


RUN echo ". /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
RUN echo ". /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

# Copy in the entrypoint script
USER root
WORKDIR /home/$USER
RUN chmod -R 666 /dev
COPY entrypoint.sh /entrypoint.sh

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set entrypoint to use the external script
ENTRYPOINT ["/entrypoint.sh"]

# Switch back to non-root user 'luna'
USER $USER
WORKDIR /home/$USER
