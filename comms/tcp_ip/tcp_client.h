#pragma once

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "comms_utils.h"
#include "serialization.h"
#include "tcp_peer.h"

namespace comms {

template <typename SendMessageType, typename RecvMessageType>
class TcpClient : public TcpPeer<SendMessageType, RecvMessageType> {
    // for templated inheritance
    using TcpPeer<SendMessageType, RecvMessageType>::config_;
    using TcpPeer<SendMessageType, RecvMessageType>::recv_buffer_;
    using TcpPeer<SendMessageType, RecvMessageType>::tcp_data_;

public:
    TcpClient(const TcpConfig& config)
    {
        config_ = config;
    }

    bool initialize()
    {
        if (client_socket_initialize() == false) {
            return false;
        }

        if (control::program_exit() == true) {
            return false;
        }

        recv_buffer_.start_recv_thread(tcp_data_.connected_sockfd);

        std::cout << "initialization done" << std::endl;
        return true;
    }

private:
    bool client_socket_initialize()
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
        for (p = servinfo; p != NULL; p = p->ai_next) {
            if ((sockfd = socket(p->ai_family, p->ai_socktype,
                     p->ai_protocol))
                == -1) {
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

        inet_ntop(p->ai_family, get_in_addr((struct sockaddr*)p->ai_addr),
            s, sizeof s);
        printf("client: connecting to %s\n", s);

        freeaddrinfo(servinfo); // all done with this structure

        tcp_data_.connected_sockfd = sockfd;

        return true;
    }
};
}