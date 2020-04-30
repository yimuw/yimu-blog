cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

project(lucas-kanada)
find_package( OpenCV REQUIRED )

add_executable(face-alignment main.cpp)
target_link_libraries(face-alignment
                      ${OpenCV_LIBS})

