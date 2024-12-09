cmake_minimum_required(VERSION 3.8)
project(zoe2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "driving/msg/DriveArc.msg"
  "driving/srv/DriveCommand.srv"
  "driving/msg/DrivePlan.msg"
  "driving/srv/GenerateDrivePlan.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
