cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

project(comms)

add_library(tcp_ip SHARED
    comms_utils.cc
)

add_subdirectory(send_number)
add_subdirectory(play_video)
