cmake_minimum_required(VERSION 3.5)
project(moon_services)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DigDump.srv"
  DEPENDENCIES std_msgs
)

ament_package()
