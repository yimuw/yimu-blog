#pragma once

#include <arpa/inet.h>
#include <condition_variable>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "comms_utils.h"
#include "serialization.h"
#include "tcp_recv_buffer.h"
#include "tcp_send_buffer.h"

namespace comms {
// C programmers are crazy...
using Socket = int;

struct TcpData {
    Socket server_sockfd;
    Socket connected_sockfd;
};

struct TcpConfig {
    std::string port;
    std::string ip;
};

template <typename SendMessageType, typename RecvMessageType>
class TcpPeer {
public:
    ~TcpPeer()
    {
        // TODO: check what will happen if close a uninitialized socket.
        close(tcp_data_.server_sockfd);
        close(tcp_data_.connected_sockfd);
    }

    bool send_to_peer(const SendMessageType& message)
    {
        // TODO: direct serialize to send buffer
        char buffer[message::size_of_message<SendMessageType>()];
        message::serialize<SendMessageType>(message, buffer);

        return send_buffer_.write_to_buff_and_trige_send(buffer, tcp_data_.connected_sockfd);
    }

    // actual recv is handled by a background thread.
    bool recv_from_peer(RecvMessageType& message)
    {
        char buffer[message::size_of_message<RecvMessageType>()];
        bool status = recv_buffer_.read(buffer);
        if (status == false) {
            return false;
        } else {
            // TODO: deserialize on the fly
            message::deserialize<RecvMessageType>(buffer, message);
            return true;
        }
    }

    TcpRecvBuffer<message::size_of_message<RecvMessageType>()> recv_buffer_;
    TcpSendBuffer<message::size_of_message<SendMessageType>()> send_buffer_;

    TcpData tcp_data_;
    TcpConfig config_;
};
}