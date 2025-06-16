cmake_minimum_required(VERSION 3.8)
project(zoe_motor_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(zoe2_hardware REQUIRED)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  zoe_motor_hardware
  SHARED
  src/rs232.cpp
  src/can.cpp
  src/commander.cpp
  src/zoe_motor_hardware.cpp
)
target_compile_features(zoe_motor_hardware PUBLIC cxx_std_17)
target_include_directories(zoe_motor_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/zoe_motor_hardware>
)
ament_target_dependencies(
  zoe_motor_hardware PUBLIC
  zoe2_hardware
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# target_link_libraries(zoe_motor_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface zoe_motor_hardware_plugin.xml)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# include_directories(include)
# add_library(zoe_motor_hardware src/)

install(
  DIRECTORY include/
  DESTINATION include/zoe_motor_hardware
)
install(TARGETS zoe_motor_hardware
  EXPORT export_zoe_motor_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


ament_export_targets(export_zoe_motor_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()