find_package( OpenCV REQUIRED )

add_executable(play_video_server
               server.cc)
target_link_libraries(play_video_server 
                      pthread
                      ${OpenCV_LIBS})

add_executable(play_video_client
               client.cc)
target_link_libraries(play_video_client 
                      pthread
                      ${OpenCV_LIBS})
