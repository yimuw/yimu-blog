cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)

project(lucas-kanada)
find_package( OpenCV REQUIRED )

add_subdirectory(lk_translation2d)
add_subdirectory(lk_se2)

