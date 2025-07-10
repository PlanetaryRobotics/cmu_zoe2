cmake_minimum_required(VERSION 3.5)
project(sim_tf)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Add the executable
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)

# Link libraries
ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  tf2_ros
  nav_msgs
)

# Install the executable
install(TARGETS
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
