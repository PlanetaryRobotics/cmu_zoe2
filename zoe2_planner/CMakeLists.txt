cmake_minimum_required(VERSION 3.5)
project(zoe2_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(zoe2_interfaces REQUIRED)

# Add the executable
add_executable(a_star_planner_node src/a_star_planner_node.cpp)
ament_target_dependencies(a_star_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  zoe2_interfaces
)

# Install the executable
install(TARGETS
  a_star_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
