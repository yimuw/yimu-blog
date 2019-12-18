add_executable(client_number
               client_number.cc)
target_link_libraries(client_number 
                      tcp_ip
                      pthread)

add_executable(server_number
               server_number.cc)
target_link_libraries(server_number 
                      tcp_ip
                      pthread)
