cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

project(3dtree)

add_executable(3dtree
               main.cc)

add_subdirectory(third_party)