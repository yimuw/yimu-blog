cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

include_directories( /usr/include/suitesparse )
include_directories( /usr/include/eigen3 )

project(ct-reconstruction)
find_package( OpenCV REQUIRED )

add_executable(ct_reconstruction main.cpp)
target_link_libraries(ct_reconstruction
                      ${OpenCV_LIBS})


