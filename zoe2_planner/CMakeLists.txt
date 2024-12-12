cmake_minimum_required(VERSION 3.5)
project(zoe2_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(zoe2_interfaces REQUIRED)
find_package(tf2 REQUIRED)

# Add include directories include/zoe2_planner
include_directories(include)


# Add the executable
add_executable(a_star_planner_node src/a_star_planner_node.cpp)
ament_target_dependencies(a_star_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  zoe2_interfaces
)

# Add the executable
add_executable(astar_server src/astar_server.cpp src/zoe2_astar_planner.cpp)
ament_target_dependencies(astar_server
  rclcpp
  geometry_msgs
  nav_msgs
  zoe2_interfaces
  tf2
)

# Install the executable
install(TARGETS
  a_star_planner_node
  astar_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
