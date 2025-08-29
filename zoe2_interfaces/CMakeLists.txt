cmake_minimum_required(VERSION 3.8)
project(zoe2_interfaces)

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Write new interfaces and include the relative path below, as a string. Example:
  msg/DriveCmd.msg
)

ament_package()
