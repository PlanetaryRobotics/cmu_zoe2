# Specify the minimum CMake version required
cmake_minimum_required(VERSION 3.12)

# Set the project name and version
project(zoe2_planner)

# Specify the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable target
add_executable(AStarPlanner zoe2_astar_planner.cpp)
add_executable(RRTPlanner zoe2_rrt_planner.cpp)