#!/bin/bash
set -e

# Default values
ROS_DISTRO="humble"
IMAGE_NAME="luna/ros2:$ROS_DISTRO"
MASTER_HOSTNAME="nuc"
IP=""
MASTER_IP=""
ROS_MASTER_PORT=11311
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

# Function to show usage
usage() {
  echo "Usage: $0 [--start (-s) | --teleop (-t) | --usb-cam (-u) | --video-stream (-v) |"
  echo "           --motor-control (-mc) | --nav (-n) | --slam (-sl) | --rviz (-r) |"
  echo "           --command (-c) <command> | --build (-b) | --stop (-x) |"
  echo "           --restart (-R) | --bash (-bsh)] [--port (-p) <port>] [--ip (-i) <host_ip>] [--master-ip (-m) <master_ip>]"
  echo "           [--master-hostname (-n) <master_hostname>] [--display (-d)] [--quiet (-q)] [--help (-h)]"
  echo "This script is used to start and manage a Docker container for ROS."
  echo "If no IP addresses are specified, the script will attempt to determine them from the hostname. If this fails, try setting the hostname or IP."
  echo "Options:"
  echo "  --start (-s)                Start all processes on the robot (motor control, navigation, SLAM)"
  echo "  --teleop (-t)               Run joystick control using teleop_launch.py"
  echo "  --usb-cam (-u)              Run usb camera node using usb_cam_launch.py"
  echo "  --video-stream (-v)         View the video stream using view_camera_launch.py"
  echo "  --motor-control (-mc)       Run motor control using motor_control_launch.py"
  echo "  --nav (-n)                  Run navigation using nav_launch.py"
  echo "  --slam (-sl)                Run SLAM using rtabmap_launch.py"
  echo "  --rviz (-z)                 Run rviz"
  echo "  --command (-c) <command>    Pass a command to be run in the container"
  echo "  --bash (-bsh)               Open a bash terminal in the container"
  echo "  --port (-p) <port>          Specify custom ROS master port (default is 11311)"
  echo "  --ip (-i) <host_ip>         Specify host IP"
  echo "  --master-ip (-m) <master_ip> Specify master IP"
  echo "  --master-hostname (-n) <master_hostname> Specify master hostname (default is raspberrypi.local)"
  echo "  --display (-d)              Enable display support (forward X11 display)"
  echo "  --build (-b)                Build the Docker container (will stop the running container if any)"
  echo "  --stop (-x)                 Stop the running Docker container"
  echo "  --restart (-r)              Restart the Docker container if it is running"
  echo "  --quiet (-q)                Suppress output"
  echo "  --help (-h)                 Show this help message"
  exit 1
}

# Parse options
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --start|-s) RUN_ROBOT_LAUNCH=true; shift ;;
    --teleop|-t) RUN_TELEOP_LAUNCH=true; shift ;;
    --usb-cam|-u) RUN_USB_CAM_NODE=true; shift ;;
    --video-stream|-v) RUN_VIEW_CAMERA_LAUNCH=true; DISPLAY_ENABLED=true; shift ;;
    --motor-control|-mc) RUN_MOTOR_CONTROL_LAUNCH=true; shift ;;
    --rviz|-z) RUN_RVIZ=true; DISPLAY_ENABLED=true; shift ;;
    --command|-c) COMMAND_TO_RUN="$2"; shift 2 ;;
    --ip|-i) IP="$2"; shift 2 ;;
    --master-ip|-m) MASTER_IP="$2"; shift 2 ;;
    --master-hostname|-n) MASTER_HOSTNAME="$2"; shift 2 ;;
    --port|-p) ROS_MASTER_PORT="$2"; shift 2 ;;
    --display|-d) DISPLAY_ENABLED=true; shift ;;
    --build|-b) BUILD_CONTAINER=true; shift ;;
    --stop|-x) STOP_CONTAINER=true; shift ;;
    --restart|-r) RESTART_CONTAINER=true; shift ;;
    --quiet|-q) QUIET_MODE=true; shift ;;
    --bash|-bsh) OPEN_BASH=true; shift ;;
    --help|-h) usage; shift ;;
    *) usage; shift ;;
  esac
done

# Ensure no conflicting launch files
if [[ "$RUN_ROBOT_LAUNCH" = true ]]; then
  if [[ "$RUN_MOTOR_CONTROL_LAUNCH" = true ]] || [[ "$RUN_NAV_LAUNCH" = true ]] || [[ "$RUN_SLAM_LAUNCH" = true ]]; then
    echo "Error: --start cannot be used with --motor-control, --nav, or --slam."
    usage
    exit 1
  fi
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

# Set up environmental variables for ros
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" > $ENV_FILE
echo "ROS_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $ENV_FILE
echo "ROS_LOCALHOST_ONLY=0" >> $ENV_FILE
echo "ROS_DISTRO=$ROS_DISTRO" >> $ENV_FILE


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

# Get the active IP address if not provided
if [[ -z "$IP" ]]; then
  IP=$(hostname -I | awk '{print $1}')
  if [[ -z "$IP" ]]; then
    echo "Warning: Unable to determine the host IP address."
  fi
fi

# Validate the IP address
if ! validate_ip "$IP"; then
  echo "Error: Invalid IP address format: $IP"
  exit 1
fi

# Determine the master IP if not provided
if [[ -z "$MASTER_IP" ]]; then
  if [[ "$(hostname)" == "$MASTER_HOSTNAME" ]]; then
    MASTER_IP=$(hostname -I | awk '{print $1}')
  else
    MASTER_IP=$(ping -c 1 $MASTER_HOSTNAME | grep 'PING' | awk -F'[()]' '{print $2}')
  fi

  if [[ -z "$MASTER_IP" ]]; then
    echo "Warning: Unable to determine the master IP address."
  fi
fi

# Validate the master IP address
if ! validate_ip "$MASTER_IP"; then
  echo "Error: Invalid master IP address format: $MASTER_IP"
  exit 1
fi

# No longer part of the Bourgeois 
DOCKER_RUN_FLAGS+=("--privileged")
DOCKER_RUN_FLAGS+=("--net=host")

# Add Docker flag to mount /dev with correct permissions
DOCKER_RUN_FLAGS+=("--volume=/dev:/dev:rw")

if [[ "$RUN_USB_CAM_NODE" = true ]]; then
  if [[ -e /dev/video0 ]]; then
    DOCKER_RUN_FLAGS+=("--device=/dev/video0")
  else
    echo "Error: /dev/video0 does not exist."
    exit 1
  fi
fi

if [[ "$RUN_SLAM_LAUNCH" = true ]]; then
  DOCKER_RUN_FLAGS+=("--device-cgroup-rule='c 13:* rmw'")
  DOCKER_RUN_FLAGS+=("--device-cgroup-rule='c 81:* rmw'")
  DOCKER_RUN_FLAGS+=("--device-cgroup-rule='c 189:* rmw'")
fi

# Enable display if requested
if [[ "$DISPLAY_ENABLED" = true ]]; then
  echo "DISPLAY=$DISPLAY" >> $ENV_FILE
  DOCKER_RUN_FLAGS+=("--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw")
  xhost +local:docker
fi

# Build the container if requested
if [[ "$BUILD_CONTAINER" = true ]]; then
  echo "Building the Docker container..."
  docker build -t $IMAGE_NAME .
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
if [[ "$QUIET_MODE" = true ]]; then
  echo "Quiet mode enabled. Suppressing output..."
  exec 1>/dev/null
  DOCKER_EXEC_FLAGS="-dt"
else
  DOCKER_EXEC_FLAGS="-it"
fi

# Execute the specified option

if [[ "$RUN_ROBOT_LAUNCH" = true ]]; then
  echo "Determining tolerable amounts of sentience..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch robot_uprising nukes_launch.py
fi

if [[ "$RUN_MOTOR_CONTROL_LAUNCH" = true ]]; then
  echo "Running motor control..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch motor_control motor_control_launch.py
fi

if [[ "$RUN_NAV_LAUNCH" = true ]]; then
  echo "Running navigation..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch navigation nav_launch.py
fi

if [[ "$RUN_SLAM_LAUNCH" = true ]]; then
  echo "Running SLAM..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch slam_config rtabmap_launch.py
fi

if [[ "$RUN_TELEOP_LAUNCH" = true ]]; then
  echo "Running joystick control..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch teleop teleop_launch.py
fi

if [[ "$RUN_USB_CAM_NODE" = true ]]; then
  echo "Running usb camera..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch usb_cam camera.launch.py
fi

if [[ "$RUN_VIEW_CAMERA_LAUNCH" = true ]]; then
  echo "Viewing ROS Camera feed..."
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh ros2 launch camera view_cam_launch.py
fi

if [[ "$RUN_RVIZ" = true ]]; then
  echo "Running rviz..."
  docker exec -it --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh rviz2
fi

if [[ -n "$COMMAND_TO_RUN" ]]; then
  echo "Running custom command: $COMMAND_TO_RUN"
  docker exec $DOCKER_EXEC_FLAGS --env-file $ENV_FILE $CONTAINER_ID /entrypoint.sh $COMMAND_TO_RUN
fi

if [[ "$OPEN_BASH" = true || "$#" -eq 0 ]]; then
  echo "Opening bash terminal..."
  docker exec -it --env-file $ENV_FILE $CONTAINER_ID bash
fi
