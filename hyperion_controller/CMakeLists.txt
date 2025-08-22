cmake_minimum_required(VERSION 3.16)
project(hyperion_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs

)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(hyperion_controller_parameters
  src/hyperion_controller_parameter.yaml
)

add_library(hyperion_controller SHARED
  src/controller.cpp
  src/hyperion_controller.cpp
)
target_compile_features(hyperion_controller PUBLIC cxx_std_17)
target_include_directories(hyperion_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/hyperion_controller>
)
target_link_libraries(hyperion_controller PUBLIC hyperion_controller_parameters)
ament_target_dependencies(hyperion_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hyperion_controller PRIVATE "HYPERION_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface hyperion_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(test_hyperion_controller
#     test/test_hyperion_controller.cpp
#     ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_hyperion_controller.yaml)
#   target_link_libraries(test_hyperion_controller
#     hyperion_controller
#   )
#   ament_target_dependencies(test_hyperion_controller
#     geometry_msgs
#     hardware_interface
#     nav_msgs
#     rclcpp
#     rclcpp_lifecycle
#     realtime_tools
#     tf2
#     tf2_msgs
#   )

#   ament_add_gmock(test_load_hyperion_controller
#     test/test_load_hyperion_controller.cpp
#   )
#   ament_target_dependencies(test_load_hyperion_controller
#     controller_manager
#     ros2_control_test_assets
#   )
endif()

install(
  DIRECTORY include/
  DESTINATION include/hyperion_controller
)
install(TARGETS hyperion_controller hyperion_controller_parameters
  EXPORT export_hyperion_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_hyperion_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
