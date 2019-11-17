cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
# enable gdb
set(CMAKE_BUILD_TYPE Debug)

project(ct-reconstruction-least-squares-common-skills)
find_package( OpenCV REQUIRED )

# sparse solver
find_package(SuiteSparse)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# TODO: why there is nothing like ${SUITE_SPARSE_LIBS}$
include_directories( /usr/include/suitesparse )
include_directories( /usr/include/eigen3 )

add_executable(ct_reconstruction_skills main.cpp)
target_link_libraries(ct_reconstruction_skills
                      ${OpenCV_LIBS}
                      cholmod)

add_executable(sparse_solver_test sparse_solver_test.cpp)
target_link_libraries(sparse_solver_test
                      cholmod)

                


