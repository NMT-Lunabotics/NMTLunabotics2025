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
RUN apt-get update && apt-get install -y \
    python3-pydantic \
    v4l-utils \
    # ros-humble-librealsense2* \
    # ros-humble-realsense2-* \
    # ros-humble-realsense2-description \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-usb-cam \
    ros-humble-image-view \
    ros-humble-image-transport-plugins \
    ros-humble-rosidl-generator-py \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-viz \
    ros-humble-rplidar-ros \
    ros-humble-imu-filter-madgwick \
    ros-humble-tf2-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-message-filters 

# RUN apt-get -y install \
#     ros-humble-robot-state-publisher \
#     ros-humble-rviz2 \
#     ros-humble-rtabmap-viz \
#     ros-humble-rplidar-ros \
#     ros-humble-imu-filter-madgwick \
#     ros-humble-tf2-ros \
#     ros-humble-message-filters

# Add the realsense cameras
USER root
RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libusb-1.0-0-dev pkg-config \
    python3-colcon-common-extensions \
    libssl-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git -b v2.55.1 && \
    mkdir -p librealsense/build && cd librealsense/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF && \
    make -j"$(nproc)" && \
    make install && ldconfig

WORKDIR /opt/librealsense
RUN ./scripts/setup_udev_rules.sh

WORKDIR /home/$USER
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development realsense-ros2 && \
    . /opt/ros/humble/setup.sh && \
    colcon build --base-paths realsense-ros2 --merge-install

# Make sure the container looks at the built realsense stuff instead of apt
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Copy in the ros workspace
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws
COPY --chown=$USER:$USER rviz2 /home/$USER/.rviz2

# Ensure the video group exists (usually does)
RUN groupadd -f video

# Add user to video group
RUN usermod -aG video $USER

# Compile the ros workspace
USER $USER
WORKDIR /home/$USER/ros2_ws
# RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build --packages-skip slam_config navigation'
# RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build --packages-skip slam_config'
RUN /bin/bash -c '. /opt/ros/humble/setup.sh; cd /home/$USER/ros2_ws; colcon build'

RUN echo ". /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
RUN echo ". /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc
RUN echo "source /home/$USER/install/setup.bash" >> /home/$USER/.bashrc

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
