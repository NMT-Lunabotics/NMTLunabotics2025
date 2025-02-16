cmake_minimum_required(VERSION 3.5)
project(moon_messages)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
    "msg/Motors.msg"
    "msg/Actuators.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install
install(
    DIRECTORY msg
    DESTINATION share/${PROJECT_NAME}
)

ament_package()