cmake_minimum_required(VERSION 3.5)
project(motor_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_raw REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node src/motor_ctrl_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs can_raw)
ament_export_dependencies(rosidl_default_runtime)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()