# Specify the minimum CMake version required
cmake_minimum_required(VERSION 3.12)

# Set the project name and version
project(zoe2_planner)

# Specify the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable target
add_executable(AStarPlanner 
		zoe2_astar_planner.cpp
		zoe2_astar_tester.cpp)
add_executable(RRTPlanner zoe2_rrt_planner.cpp)

target_include_directories(zoe2_planner
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> 
        $<INSTALL_INTERFACE:include/zoe2_astar_planner> 
)