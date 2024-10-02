# Stage 1: Build Stage
FROM osrf/ros:humble-desktop as builder

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Install build dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and copy source code
WORKDIR /ros2_ws
COPY . /ros2_ws/src

# Install dependencies and build
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Stage 2: Runtime Stage
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Copy built workspace from the builder stage
COPY --from=builder /ros2_ws /ros2_ws

# Source the ROS environment and workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["bash"]
