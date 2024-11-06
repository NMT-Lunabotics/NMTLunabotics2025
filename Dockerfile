# Stage 1: Build Stage
FROM osrf/ros:humble-desktop as builder

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=0

ARG USER=luna

# Add user and grant sudo privileges
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USER
WORKDIR /home/$USER

USER root

# Use Bash shell
SHELL ["/bin/bash", "-c"]

# Install build dependencies and your specified packages
RUN sudo apt-get update && \
    sudo apt-get install -y \
        python3-colcon-common-extensions \
        ros-humble-ament-cmake \
        ros-humble-teleop-twist-joy \
        ros-humble-joy \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-realsense2-* \
        ros-humble-rviz2 \
        ros-humble-rtabmap-ros \
        ros-humble-pointcloud-to-laserscan \

    && sudo rm -rf /var/lib/apt/lists/*

# Create workspace and copy source code
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws

# Install ROS package dependencies and build the workspace
RUN source /opt/ros/humble/setup.bash && \
    cd /home/$USER/ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Stage 2: Runtime Stage
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=0

ARG USER=luna

# Add user and grant sudo privileges
RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        && rm -rf /var/lib/apt/lists/* \
    && pip3 install cantools
RUN useradd -m $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER root
WORKDIR /home/$USER

# Use Bash shell
SHELL ["/bin/bash", "-c"]

# Install runtime dependencies
RUN apt-get update && \
    apt-get install -y \
        ros-humble-ament-cmake \
        ros-humble-teleop-twist-joy \
        ros-humble-joy \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-realsense2-* \
        ros-humble-rviz2 \
        ros-humble-rtabmap-ros \
        ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# Copy built workspace from the builder stage
COPY --from=builder /home/$USER/ros2_ws /home/$USER/ros2_ws

# Source the ROS environment and workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/$USER/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /home/$USER

CMD ["bash"]
