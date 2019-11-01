add_executable(tcp_server
               tcp_server_main.cc)
target_link_libraries(tcp_server 
                      pthread)

add_executable(client_example_while
               client_example_while.cc)
