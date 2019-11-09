#pragma once

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
#include "tcp_recv_buffer.h"
#include "tcp_sent_buffer.h"
#include "tcp_peer.h"


namespace comms
{

template<typename SendMessageType, typename RecvMessageType>
class TcpServer : public TcpPeer<SendMessageType, RecvMessageType>
{
    // for templated inherentance
    using TcpPeer<SendMessageType, RecvMessageType>::config_;
    using TcpPeer<SendMessageType, RecvMessageType>::recv_buffer_;
    using TcpPeer<SendMessageType, RecvMessageType>::tcp_data_;
public:
    TcpServer(const TcpConfig &config)
    {
        config_ = config;
    }

    bool initailize()
    {
        if(server_socket_initailization() == false)
        {
            return false;
        }

        if(wait_for_client_connection(tcp_data_.connected_sockfd) == false)
        {
            return false;
        }

        if(control::program_exit() == true)
        {
            return false;
        }

        recv_buffer_.start_recv_thread(tcp_data_.connected_sockfd);

        std::cout << "initailize done" << std::endl;
        return true;
    }

private:
    // Thanks beej!!
    // https://beej.us/guide/bgnet/html//index.html
    bool server_socket_initailization()
    {
        struct addrinfo hints;
        memset(&hints, 0, sizeof hints);
	    hints.ai_family = AF_UNSPEC;
	    hints.ai_socktype = SOCK_STREAM;

        int rv;
        struct addrinfo *servinfo;

        if(config_.ip == "AI_PASSIVE")
        {
            hints.ai_flags = AI_PASSIVE;
            if ((rv = getaddrinfo(nullptr, config_.port.c_str(), &hints, &servinfo)) != 0) 
            {
                std::cerr << "getaddrinfo: " << gai_strerror(rv) << std::endl;
                return false;
            }
        }	
        else
        {
            if ((rv = getaddrinfo(config_.ip.c_str(), config_.port.c_str(), &hints, &servinfo)) != 0) 
            {
                std::cerr << "getaddrinfo: " << gai_strerror(rv) << std::endl;
                return false;
            }
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

        tcp_data_.server_sockfd = sockfd;

        std::cout << "server initailization done. waiting for connections..." << std::endl;

        return true;
    }

    // blocking call
    bool wait_for_client_connection(Socket &new_fd)
    {
        std::cout << "waiting for client ...." << std::endl;
        struct sockaddr_storage their_addr;
        socklen_t sin_size = sizeof their_addr;
		new_fd = accept(tcp_data_.server_sockfd, (struct sockaddr *)&their_addr, &sin_size);
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
};
}