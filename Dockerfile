ARG ROS_DISTRO=humble

# Use multi-architecture support
ARG BASE_IMAGE_AMD=osrf/ros:$ROS_DISTRO-desktop-full
ARG BASE_IMAGE_ARM=arm64v8/ros:$ROS_DISTRO-ros-base

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
RUN apt-get update && apt-get install -y python3 python3-pip git curl wget

# Install python packages
RUN pip3 install pyserial

# Add error handling for packages that might not be available on ARM
RUN apt-get update && \
    apt-get -y install ros-$ROS_DISTRO-teleop-twist-joy || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-joy || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-navigation2 || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-nav2-bringup || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-realsense2-camera || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-rviz2 || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-rtabmap-ros || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-rmw-cyclonedds-cpp || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-usb-cam || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-image-view || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-image-transport-plugins || echo "Package not available" && \
    apt-get -y install python3-pydantic v4l-utils || echo "Package not available" && \
    apt-get -y install ros-$ROS_DISTRO-rosidl-generator-py || echo "Package not available"

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN mv bin/arduino-cli /usr/local/bin/arduino-cli

# Set up Arduino CLI
RUN arduino-cli config init
RUN arduino-cli core update-index
RUN arduino-cli core install arduino:avr

# Install dependencies for micro-ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    ros-$ROS_DISTRO-rosidl-generator-py

# Clone the micro-ROS setup repository
USER $USER
WORKDIR /home/$USER/microros_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies using rosdep
USER root
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build the micro-ROS tools
USER $USER
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Source the micro-ROS installation
RUN echo "source /home/$USER/microros_ws/install/local_setup.bash" >> /home/$USER/.bashrc

# Set up micro-ROS workspace
USER $USER
WORKDIR /home/$USER/micro_ros_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    ros2 run micro_ros_setup create_firmware_ws.sh && \
    ros2 run micro_ros_setup configure_firmware.sh generic_linux && \
    ros2 run micro_ros_setup build_firmware.sh"

# Copy in the ros workspace
COPY --chown=$USER:$USER ros2_ws /home/$USER/ros2_ws
COPY --chown=$USER:$USER rviz2 /home/$USER/.rviz2

# Compile the ros workspace
USER $USER
WORKDIR /home/$USER/ros2_ws
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.sh; cd /home/$USER/ros2_ws; colcon build'

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc
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
