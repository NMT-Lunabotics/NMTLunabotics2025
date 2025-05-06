# Use multi-architecture support
ARG BASE_IMAGE_AMD=osrf/ros:humble-desktop-full
ARG BASE_IMAGE_ARM=arm64v8/ros:humble-ros-base

# Define initial stages
FROM ${BASE_IMAGE_AMD} AS base_amd64
FROM ${BASE_IMAGE_ARM} AS base_arm64

# Select the appropriate base
ARG TARGETARCH
FROM base_${TARGETARCH}

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
RUN apt-get update && apt-get install -y python3 python3-pip

# Install python packages
RUN pip3 install pyserial

# Add error handling for packages that might not be available on ARM
RUN apt-get update && \
    apt-get -y install ros-humble-teleop-twist-joy || echo "Package not available" && \
    apt-get -y install ros-humble-joy || echo "Package not available" && \
    apt-get -y install ros-humble-navigation2 || echo "Package not available" && \
    apt-get -y install ros-humble-nav2-bringup || echo "Package not available" && \
    apt-get -y install ros-humble-realsense2-camera || echo "Package not available" && \
    apt-get -y install ros-humble-rviz2 || echo "Package not available" && \
    apt-get -y install ros-humble-rtabmap-ros || echo "Package not available" && \
    apt-get -y install ros-humble-rmw-cyclonedds-cpp || echo "Package not available" && \
    apt-get -y install ros-humble-usb-cam || echo "Package not available" && \
    apt-get -y install ros-humble-image-view || echo "Package not available" && \
    apt-get -y install ros-humble-image-transport-plugins || echo "Package not available" && \
    apt-get -y install python3-pydantic v4l-utils || echo "Package not available" && \
    apt-get -y install ros-humble-rosidl-generator-py || echo "Package not available"

RUN apt-get -y install \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rtabmap-viz \
    ros-humble-rplidar-ros \
    ros-humble-imu-filter-madgwick \
    ros-humble-tf2-ros \
    ros-humble-message-filters \


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
