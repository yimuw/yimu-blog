cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall)

project(rocket-landing)

find_package(PythonLibs 2.7)

add_executable(rocket_landing main.cpp)
target_include_directories(rocket_landing PRIVATE 
                           ${PYTHON_INCLUDE_DIRS})
target_link_libraries(rocket_landing 
                      ${PYTHON_LIBRARIES})


add_executable(test_rocket_landing_solver solver_tests.cpp)



