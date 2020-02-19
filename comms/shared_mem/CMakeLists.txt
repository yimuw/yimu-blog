project(naive_comms)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# make sure the code work with O3 optimization
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package( OpenCV REQUIRED )


add_executable(reader1 
               reader1.cc)
target_link_libraries(reader1 
					  ${OpenCV_LIBS}
					  rt) 

add_executable(reader2 
               reader2.cc)
target_link_libraries(reader2 
                      ${OpenCV_LIBS}
					  rt) 


add_executable(writer
               writer.cc)
target_link_libraries(writer 
                      ${OpenCV_LIBS}
					  rt) 

