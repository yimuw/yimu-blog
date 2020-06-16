cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

include_directories( /usr/include/eigen3 )

project(matrix-mul)

add_executable(tests 
               tests/tests.cc)

