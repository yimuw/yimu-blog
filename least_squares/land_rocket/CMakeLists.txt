cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall)

project(rocket-landing)

add_executable(rocket_landing main.cpp)
# target_link_libraries(rocket_landing)


