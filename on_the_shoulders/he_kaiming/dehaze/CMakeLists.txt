cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall -O2)

project(dehaze)
find_package( OpenCV REQUIRED )

add_executable(dehaze main.cpp)
target_link_libraries(dehaze
                      ${OpenCV_LIBS})
