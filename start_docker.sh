#!/bin/bash
set -e

# Default values
ROS_DISTRO="humble"
IMAGE_NAME="luna/ros2:$ROS_DISTRO"
# MASTER_HOSTNAME="nuc"
# IP=""
# MASTER_IP=""
# ROS_MASTER_PORT=11311
DISPLAY_ENABLED=false
DOCKER_RUN_FLAGS=()
COMMAND_TO_RUN=""
ENV_FILE="env_file.txt"
BUILD_CONTAINER=false
STOP_CONTAINER=false
RESTART_CONTAINER=false
QUIET_MODE=false
ROS_DOMAIN_ID=42
OPEN_BASH=false
RUN_RVIZ=false

RUN_MAPPING=false
RUN_NAV=false
SAVE_MAP=false
COPY_MAP=false

usage() {
    echo "Usage: $0 [--start (-s) | --motor_ctrl (-m) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |"
    echo "           --mapping (-M) | --nav (-n) | --save-map (-S) | --copy-map (-C) |"
    echo "           --command (-c) <command> | --build (-b) | --stop (-x) |"
    echo "           --restart (-r)] [--display (-d)] [--quiet (-q)] [--ros-domain-id (-i) <id>] [--help (-h)]"
    echo "This script is used to start and manage a Docker container for ROS2"
    echo "If no action is specified, the script will open an interactive bash terminal in the container."
    echo "Actions (pick ONE):"
    echo "  --start (-s)                Start all processes on the robot"
    echo "  --motor_ctrl (-m)           Run motor control"
    echo "  --teleop (-t)               Run joystick control using teleop.launch"
    echo "  --usb-cam (-u)              Run usb camera node using usb_cam.launch"
    echo "  --video-stream (-v)         View the video stream using view_camera.launch"
    echo "  --mapping (-M)              Run rtabmap_launch.py from slam_config"
    echo "  --nav (-n)                  Run nav_launch.py from navigation"
    echo "  --save-map (-S)             Save map using nav2_map_server map_saver_cli"
    echo "  --copy-map (-C)             Copy map files from Docker to ./maps directory"
    echo "  --command (-c) <command>    Pass a command to be run in the container"
    echo "Options:"
    echo "  --display (-d)              Enable display support (forward X11 display)"
    echo "  --build (-b)                Build the Docker container (will stop the running container if any)"
    echo "  --stop (-x)                 Stop the running Docker container"
    echo "  --restart (-r)              Restart the Docker container if it is running"
    echo "  --quiet (-q)                Suppress output"
    echo "  --ros-domain-id (-i) <id>   Set the ROS_DOMAIN_ID (default: 42)"
    echo "  --help (-h)                 Show this help message"
    exit 1
}

# Parse options
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --start|-s) RUN_ROBOT_LAUNCH=true; shift ;;
        --motor_ctrl|-m) RUN_MOTOR_CTRL=true; shift ;;
        --teleop|-t) RUN_TELEOP_LAUNCH=true; shift ;;
        --usb-cam|-u) RUN_USB_CAM_NODE=true; shift ;;
        --video-stream|-v) RUN_VIEW_CAMERA_LAUNCH=true; DISPLAY_ENABLED=true; shift ;;
        --mapping|-M) RUN_MAPPING=true; shift ;;
        --nav|-n) RUN_NAV=true; shift ;;
        --save-map|-S) SAVE_MAP=true; shift ;;
        --copy-map|-C) COPY_MAP=true; shift ;;
        --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
        --display|-d) DISPLAY_ENABLED=true; shift ;;
        --build|-b) BUILD_CONTAINER=true; shift ;;
        --stop|-x) STOP_CONTAINER=true; shift ;;
        --restart|-r) RESTART_CONTAINER=true; shift ;;
        --quiet|-q) QUIET_MODE=true; shift ;;
        --ros-domain-id|-i) ROS_DOMAIN_ID="$2"; shift 2 ;;
        --help|-h) usage; shift ;;
        *) usage; shift ;;
    esac
done

# Check if multiple actions are specified
ACTION_COUNT=0
if [ "$RUN_ROBOT_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_MOTOR_CTRL" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_TELEOP_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_USB_CAM_NODE" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_VIEW_CAMERA_LAUNCH" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_MAPPING" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$RUN_NAV" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$SAVE_MAP" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ "$COPY_MAP" = true ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi
if [ -n "$COMMAND_TO_RUN" ]; then ACTION_COUNT=$((ACTION_COUNT + 1)); fi

if [ "$ACTION_COUNT" -gt 1 ]; then
    echo "Error: Multiple actions specified. You get ONE."
    usage
    exit 1
fi

# Check if the container is already running
RUNNING=false
if docker ps -q -f ancestor=$IMAGE_NAME | grep -q .; then
    echo "Docker container is running."
    RUNNING=true
else
    echo "Docker container is not running."
fi

# Stop or restart the container if requested
if [[ "$STOP_CONTAINER" = true || "$RESTART_CONTAINER" = true  || "$BUILD_CONTAINER" = true ]]; then
    if [[ "$RUNNING" = true ]]; then
        echo "Stopping the running Docker container..."
        docker stop $(docker ps -q -f ancestor=$IMAGE_NAME)
        RUNNING=false
    else
        echo "Nothing to stop."
    fi

    if [[ "$STOP_CONTAINER" = true ]]; then
        echo "Exiting..."
        exit 0
    fi
fi

# Function to validate IP address
validate_ip() {
    local ip=$1
    local stat=1

    if [[ $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        OIFS=$IFS
        IFS='.'
        ip=($ip)
        IFS=$OIFS
        [[ ${ip[0]} -le 255 && ${ip[1]} -le 255 && ${ip[2]} -le 255 && ${ip[3]} -le 255 ]]
        stat=$?
    fi
    return $stat
}

# # Get the active IP address if not provided
# if [[ -z "$IP" ]]; then
#     IP=$(hostname -I | awk '{print $1}')
#     if [[ -z "$IP" ]]; then
#         echo "Error: Unable to determine the host IP address."
#         # exit 1
#     fi
# fi

# # Validate the IP address
# if ! validate_ip "$IP"; then
#     echo "Error: Invalid IP address format: $IP"
#     # exit 1
# fi

# # Determine the master IP if not provided
# if [[ -z "$MASTER_IP" ]]; then
#     if [[ "$(hostname)" == "$MASTER_HOSTNAME" ]]; then
#         MASTER_IP=$(hostname -I | awk '{print $1}')
#     else
#         MASTER_IP=$(ping -c 1 $MASTER_HOSTNAME | grep 'PING' | awk -F'[()]' '{print $2}')
#     fi

#     if [[ -z "$MASTER_IP" ]]; then
#         echo "Error: Unable to determine the master IP address."
#         # exit 1
#     fi
# fi

# # Validate the master IP address
# if ! validate_ip "$MASTER_IP"; then
#     echo "Error: Invalid master IP address format: $MASTER_IP"
#     # exit 1
# fi

# Set ROS_DOMAIN_ID
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" > $ENV_FILE

# No longer part of the Bourgeois 
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")

# Add Docker flag to mount /dev with correct permissions
DOCKER_RUN_FLAGS+=("--volume=/dev:/dev:rw")

if [ "$RUN_USB_CAM_NODE" = true ]; then
    if [ -e /dev/video0 ]; then
        DOCKER_RUN_FLAGS+=("--device=/dev/video0")
    else
        echo "Error: /dev/video0 does not exist."
        exit 1
    fi
fi

# Enable display if requested
if [[ "$DISPLAY_ENABLED" = true ]]; then
    echo "DISPLAY=$DISPLAY" >> $ENV_FILE
    DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
    xhost +local:docker
fi

# Build the container if requested
if [[ "$BUILD_CONTAINER" = true ]]; then
    echo "Building the Docker container for the current architecture..."
    ARCH=$(uname -m)
    TARGET_ARCH=""
    if [[ "$ARCH" == "x86_64" ]]; then
        echo "Detected architecture: AMD64"
        TARGET_ARCH="amd64"
    elif [[ "$ARCH" == "aarch64" ]]; then
        echo "Detected architecture: ARM64"
        TARGET_ARCH="arm64"
    else
        echo "Error: Unsupported architecture: $ARCH"
        exit 1
    fi

    docker build -t $IMAGE_NAME --build-arg TARGET_ARCH=$TARGET_ARCH .
fi

# Start the container if it is not already running
if [[ "$RUNNING" = false ]]; then
    echo "Starting the Docker container..."
    docker run -dit --env-file $ENV_FILE "${DOCKER_RUN_FLAGS[@]}" $IMAGE_NAME bash
fi

CONTAINER_ID=$(docker ps -q -f ancestor=$IMAGE_NAME) # Get container ID
CONTAINER_ID=$(echo "$CONTAINER_ID" | xargs) # Trim whitespace
echo "Container ID: $CONTAINER_ID"

# Check if the container ID is empty
if [[ -z "$CONTAINER_ID" ]]; then
    echo "Error: Failed to start the Docker container."
    exit 1
fi

# Run docker commands in detached mode if quiet mode is enabled
if [ "$QUIET_MODE" = true ]; then
    echo "Quiet mode enabled. Suppressing output..."
    exec 1>/dev/null
    DOCKER_EXEC_FLAGS="-dt"
else
    DOCKER_EXEC_FLAGS="-it"
fi

# Execute the specified option

if [ "$RUN_ROBOT_LAUNCH" = true ]; then
    echo "Determining tolerable amounts of sentience..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch robot_uprising nukes_launch.py
elif [ "$RUN_MOTOR_CTRL" = true ]; then
    echo "Running motor control..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch motor_control motor_control_launch.py
elif [ "$RUN_TELEOP_LAUNCH" = true ]; then
    echo "Running joystick control..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch teleop teleop_launch.py
elif [ "$RUN_USB_CAM_NODE" = true ]; then
    echo "Running usb camera..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch camera usb_cam_launch.py
elif [ "$RUN_VIEW_CAMERA_LAUNCH" = true ]; then
    echo "Viewing ROS Camera feed..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch camera view_cam_launch.py
elif [ "$RUN_MAPPING" = true ]; then
    echo "Running rtabmap mapping (slam_config/rtabmap_launch.py)..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch slam_config rtabmap_launch.py
elif [ "$RUN_NAV" = true ]; then
    echo "Running navigation (navigation/nav_launch.py)..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch navigation nav_launch.py
elif [ "$SAVE_MAP" = true ]; then
    echo "Saving map to ~/my_map..."
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args --remap map:=/rtabmap/map
    echo "Copying map files from Docker to ./maps directory..."
    mkdir -p ./maps
    docker cp $CONTAINER_ID:/home/luna/my_map.pgm ./maps/my_map.pgm
    docker cp $CONTAINER_ID:/home/luna/my_map.yaml ./maps/my_map.yaml
    echo "Map files copied to ./maps/"
elif [ -n "$COMMAND_TO_RUN" ]; then
    echo "Running custom command: $COMMAND_TO_RUN"
    docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh $COMMAND_TO_RUN
else
    echo "No options specified. Opening interactive bash terminal in the container..."
    docker exec -it --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh bash
fi
