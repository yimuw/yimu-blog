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

struct TcpClientConfig
{
    std::string port;
    std::string ip;
};

template<size_t CellSizeByte>
class TcpClient
{
public:
    // C programmers are crazy...
    using Socket = int;
    struct TcpData
    {
        Socket sockfd;
    };

    TcpClient(const TcpClientConfig &config)
        : config_(config)
    {
    }

    ~TcpClient()
    {
        close(tcp_data_.sockfd);

        if(data_handing_thread_.joinable())
        {
            std::cout << "joining thread..." << std::endl;
            data_handing_thread_.join();
            std::cout << "data_handing_thread joined" << std::endl;
        }
    }

    bool initailize()
    {
        data_handing_thread_ = std::thread([this]()
        {
            std::cout << "interal_loop start" << std::endl;
            this->interal_loop();
            std::cout << "interal_loop end" << std::endl;
        });

        std::cout << "initailize done" << std::endl;
        return true;
    }

    bool has_data()
    {
        return buffer_.has_data();
    }

    bool read(char * const target_data_ptr)
    {
        auto copy_to_buffer = [&target_data_ptr](char const * const src_data_ptr)
        {
            memcpy(target_data_ptr, src_data_ptr, CellSizeByte);
            return true;
        };

        const bool status = buffer_.process(copy_to_buffer);
        return status;
    }

private:
    bool socket_initailization()
    {
        int sockfd;
        struct addrinfo hints, *servinfo, *p;
        int rv;
        char s[INET6_ADDRSTRLEN];
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;

        if ((rv = getaddrinfo(config_.ip.c_str(), config_.port.c_str(), &hints, &servinfo)) != 0) {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
            return false;
        }

        // loop through all the results and connect to the first we can
        for(p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype,
                    p->ai_protocol)) == -1) {
                perror("client: socket");
                continue;
            }

            if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
                perror("client: connect");
                close(sockfd);
                continue;
            }

            break;
        }

        if (p == NULL) {
            fprintf(stderr, "client: failed to connect\n");
            return false;
        }

        inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
                s, sizeof s);
        printf("client: connecting to %s\n", s);

        freeaddrinfo(servinfo); // all done with this structure

        tcp_data_.sockfd = sockfd;

        return true;
    }

    void interal_loop()
    {
        // loop for new connections
        while(control::program_exit() == false)
        {
            if(socket_initailization() == false)
            {
                break;
            }
            
            recv_data_and_write_to_queue(tcp_data_.sockfd);
            close(tcp_data_.sockfd);
            std::cout << "lost connection, retry..." << std::endl;
            sleep(1);
        }
    }

    // TODO: how to do it on client side? current handled by n = recv 
    bool check_socket_connection(Socket connected_client)
    {
        return true;
    }

    void recv_data_and_write_to_queue(Socket connected_client)
    {
        // TODO: super large array in stack is not good.
        char buf[CellSizeByte];

        while(true)
        {
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

            int received_data = 0;
            namespace sync = comms::package_sync;
            sync::SyncStatus status = sync::wait_for_control_packge(connected_client, buf, received_data);
            if(status == sync::SyncStatus::success)
            {
                const int rest_bytes = CellSizeByte - received_data;
                assert(rest_bytes >= 0);
                if(rest_bytes > 0)
                {
                    bool recv_status = recv_all(connected_client, buf + received_data, 
                        CellSizeByte - received_data);
                    if (recv_status == false)
                    {
                        perror("recv_all failed");
                        std::cerr << "recv_all failed" << std::endl;
                        break;
                    }
                }
    
                // TODO: not efficient. Do one copy.
                buffer_.write(buf);
            }
            else if(status == sync::SyncStatus::failure)
            {
                break;
            }
            
        }
    }

    // data handling thread
    std::thread data_handing_thread_;

    static constexpr size_t BUFFER_LENGTH {10};
    CircularBuffer<BUFFER_LENGTH, CellSizeByte> buffer_;

    TcpData tcp_data_;
    TcpClientConfig config_;
};
}