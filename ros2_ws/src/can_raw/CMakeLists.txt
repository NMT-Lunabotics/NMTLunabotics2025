cmake_minimum_required(VERSION 3.5)
project(can_raw)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/CanFrame.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

include_directories(include)

add_executable(can_raw_node src/can_raw_node.cpp)
ament_target_dependencies(can_raw_node rclcpp std_msgs)

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS can_raw_node
    DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(can_raw_node ${cpp_typesupport_target})

ament_package()
