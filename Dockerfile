###############################################################################
# 0. Multi-arch ROS Humble base
###############################################################################
ARG BASE_IMAGE_AMD=osrf/ros:humble-desktop-full
ARG BASE_IMAGE_ARM=arm64v8/ros:humble-ros-base

FROM ${BASE_IMAGE_AMD} AS base_amd64
FROM ${BASE_IMAGE_ARM} AS base_arm64

ARG TARGETARCH
FROM base_${TARGETARCH}

###############################################################################
# 1. Create user “luna” with sudo + dialout + video
###############################################################################
ARG USER=luna
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y sudo \
 && useradd -m ${USER} \
 && echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
 && usermod -aG dialout,video ${USER}

###############################################################################
# 2. Common runtime & build dependencies
###############################################################################
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-pydantic v4l-utils \
    git cmake build-essential \
    libusb-1.0-0-dev pkg-config \
    python3-colcon-common-extensions \
    libssl-dev \
    libglfw3-dev \ 
    libgl1-mesa-dev \
    libglu1-mesa-dev \
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
    ros-humble-message-filters && \
    pip3 install --no-cache-dir pyserial

RUN apt-get remove -y 'librealsense2*'


###############################################################################
# 3. Build librealsense (RSUSB backend - no kernel patch required)
###############################################################################
WORKDIR /opt
RUN git clone -b v2.56.1 https://github.com/IntelRealSense/librealsense.git && \
    mkdir librealsense/build && cd librealsense/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
             -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF && \
    make -j$(nproc) && make install && ldconfig

# copy the udev rule (no need to run the helper that expects hardware)
WORKDIR /opt/librealsense
RUN mkdir -p /etc/udev/rules.d && \
    cp config/99-realsense-libusb.rules /etc/udev/rules.d/

ENV LD_LIBRARY_PATH=/usr/local/lib

###############################################################################
# 4. Clone realsense-ros2 into the workspace
###############################################################################
WORKDIR /home/${USER}/ros2_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git

###############################################################################
# 5. Copy your own packages & RViz config
###############################################################################
WORKDIR /home/${USER}
COPY --chown=${USER}:${USER} ros2_ws  /home/${USER}/ros2_ws
COPY --chown=${USER}:${USER} rviz2    /home/${USER}/.rviz2

###############################################################################
# 6. Prepare log/ folder so colcon (non-root) can write there
###############################################################################
RUN mkdir -p /home/${USER}/ros2_ws/log && \
    chown -R ${USER}:${USER} /home/${USER}/ros2_ws

###############################################################################
# 7. Build the complete workspace as user luna
###############################################################################
USER ${USER}
WORKDIR /home/${USER}/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install               

###############################################################################
# 8. Convenience sourcing for interactive shells
###############################################################################
RUN echo 'source /opt/ros/humble/setup.bash'         >> /home/${USER}/.bashrc && \
    echo 'source ~/ros2_ws/install/setup.bash' >> /home/${USER}/.bashrc

###############################################################################
# 9. Entrypoint
###############################################################################
USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default shell context
USER ${USER}
WORKDIR /home/${USER}