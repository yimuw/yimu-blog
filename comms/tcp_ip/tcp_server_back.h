#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

#include <string>
#include <iostream>
#include <thread>             
#include <mutex>              
#include <condition_variable>

#include "comms_utils.h"


namespace comms
{

struct TcpServerConfig
{
    std::string port;
    std::string ip;
};

template<size_t CellSizeByte>
class TcpServer
{
public:
    // C programmers are crazy...
    using Socket = int;
    struct TcpData
    {
        Socket sockfd;
    };

    TcpServer(const TcpServerConfig &config)
        : config_(config)
    {
    }

    ~TcpServer()
    {
        close(tcp_data_.sockfd);
        cv.notify_one();

        if(data_handing_thread_.joinable())
        {
            std::cout << "joining thread..." << std::endl;
            data_handing_thread_.join();
            std::cout << "data_handing_thread joined" << std::endl;
        }
    }

    bool initailize()
    {
        if(socket_initailization() == false)
        {
            return false;
        }

        // Start the background thread
        data_handing_thread_ = std::thread([this]()
        {
            std::cout << "interal_loop start" << std::endl;
            this->interal_loop();
        });

        std::cout << "initailize done" << std::endl;
        return true;
    }

    // Copy to buffer, and notify data handing thread to call tcp send.
    // Not guarantee to publish successfully. 
    bool publish(char const * const data_ptr)
    {
        // copy data to buffer
        if(buffer_.write(data_ptr))
        {
            // notice data_handing_thread_ to send by tcp.
            cv.notify_one();
            return true;
        }
        else
        {
            std::cout << "write failed!" << std::endl;
            return false;
        } 
    }

private:
    // Thanks beej!!
    // https://beej.us/guide/bgnet/html//index.html
    bool socket_initailization()
    {
        struct addrinfo hints;
        memset(&hints, 0, sizeof hints);
	    hints.ai_family = AF_UNSPEC;
	    hints.ai_socktype = SOCK_STREAM;

        int rv;
        struct addrinfo *servinfo;
        if ((rv = getaddrinfo(config_.ip.c_str(), config_.port.c_str(), &hints, &servinfo)) != 0) 
        {
            std::cerr << "getaddrinfo: " << gai_strerror(rv) << std::endl;
			return false;
		}	

        struct addrinfo *p;
        Socket sockfd;
        int yes=1;
        // loop through all the results and bind to the first we can
	    for(p = servinfo; p != NULL; p = p->ai_next) 
        {
            if ((sockfd = socket(p->ai_family, p->ai_socktype,
                    p->ai_protocol)) == -1) 
            {
                std::cerr << "server: socket" << std::endl;
                continue;
            }

            if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                    sizeof(int)) == -1) 
                    {
                std::cerr << "setsockopt" << std::endl;
                return false;
            }

            if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) 
            {
                close(sockfd);
                std::cerr << "server: bind" << std::endl;
                continue;
            }

            break;
        }

        char s[INET6_ADDRSTRLEN];
        inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
			s, sizeof s);
        std::cout << "server address: " << s << std::endl;

	    freeaddrinfo(servinfo); // all done with this structure

        if (p == NULL)  
        {
            std::cerr << "server: failed to bind" << std::endl;
            return false;
        }

        constexpr int BACKLOG = 10;
        if (listen(sockfd, BACKLOG) == -1) 
        {
            std::cerr << "listen" << std::endl;
            return false;
        }

        if(kill_dead_processes() == false)
        {
            std::cerr << "kill_dead_processes" << std::endl;
            return false;
        }

        // non-blocking
        if(fcntl(sockfd, F_SETFL, O_NONBLOCK) == -1)
        {
            std::cerr << "fcntl failed" << std::endl;
            return false;
        }
        tcp_data_.sockfd = sockfd;

        std::cout << "server initailization done. waiting for connections..." << std::endl;

        return true;
    }

    // Maybe is shouldn't be a class method
    bool kill_dead_processes()
    {
        struct sigaction sa;
        sa.sa_handler = sigchld_handler; // reap all dead processes
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        if (sigaction(SIGCHLD, &sa, NULL) == -1) 
        {
            perror("sigaction");
            return false;
        }

        return true;
    }

    bool wait_for_connection(Socket &new_fd)
    {
        struct sockaddr_storage their_addr;
        socklen_t sin_size = sizeof their_addr;
		new_fd = accept(tcp_data_.sockfd, (struct sockaddr *)&their_addr, &sin_size);
		if (new_fd == -1) 
        {
			perror("accept");
			return false;
		}

        char s[INET6_ADDRSTRLEN];
		inet_ntop(their_addr.ss_family,
			get_in_addr((struct sockaddr *)&their_addr),
			s, sizeof s);
		printf("server: got connection from %s\n", s);
		return true;
    }

    void interal_loop()
    {
        // loop for new connections
        while(control::program_exit() == false)
        {
            // have a connection!
            // It is a blocking call
            Socket new_fd;
            if(wait_for_connection(new_fd) == false)
            {
                std::cout << "no connection. retry" << std::endl;
                sleep(1);
                if(control::program_exit())
                {
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                std::cout << "get connection" << std::endl;

                // send data to client
                // It is a blocking call
                send_data_in_queue_to_client(new_fd);

                std::cout << "close socket" << std::endl;
                close(new_fd);
            }
        }
    }

    bool check_socket_connection(Socket connected_client)
    {
        int error_code;
        socklen_t error_code_size = sizeof(error_code);
        if(getsockopt(connected_client, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size) == -1)
        {
            std::cout << "getsockopt failed" << std::endl;
            return false;
        }
        else if(error_code != 0)
        {
            std::cout << "error_code: " << error_code << std::endl;
            return false;
        }

        return true;
    }

    void send_data_in_queue_to_client(Socket connected_client)
    {
        while(true)
        {
            std::unique_lock<std::mutex> lck(mtx);
            cv.wait(lck);

            if(control::program_exit() == true)
            {
                std::cout << "exit send_data_in_queue_to_client loop" << std::endl;
                break;
            }

            // connect to socket
            if(check_socket_connection(connected_client) == false)
            {
                break;
            }

            package_sync::send_control_package(connected_client);

            auto icp_send_function = [&connected_client](char * const data_ptr)
            {
                std::cout << "send data by icp::send"  << std::endl;

                int status = sendall(connected_client, data_ptr, CellSizeByte);
                if(status == -1)
                {
                    std::cout << "send failed" << std::endl;
                    return false;
                }
                else
                {
                    return true;
                }
            };

            // send everything in buffer
            while(buffer_.process(icp_send_function) == true);
        }
    }

    // threads comms
    std::mutex mtx;
    std::condition_variable cv;

    // data handling thread
    std::thread data_handing_thread_;

    static constexpr size_t BUFFER_LENGTH {10};
    CircularBuffer<BUFFER_LENGTH, CellSizeByte> buffer_;

    TcpData tcp_data_;

    TcpServerConfig config_;
};
}