cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-g -Wall)

project(rocket-landing)

include_directories( /usr/include/suitesparse )
include_directories( /usr/include/eigen3 )

# tricky with anaconda
# TODO: docker
find_package(PythonLibs 3.5)

add_executable(rocket_landing 
               main.cpp)
target_include_directories(rocket_landing PRIVATE
                           ${PYTHON_INCLUDE_DIRS})
target_link_libraries(rocket_landing 
                      ${PYTHON_LIBRARIES}
                      cholmod)


add_executable(test_rocket_landing_solver 
               solver_tests.cpp)
target_link_libraries(test_rocket_landing_solver 
                      cholmod)





