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


namespace comms
{
// C programmers are crazy...
using Socket = int;

struct TcpData
{
    Socket server_sockfd;
    Socket connected_sockfd;
};

struct TcpConfig
{
    std::string port;
    std::string ip;
};

template<typename MessageType>
class TcpPeer
{
public:
    ~TcpPeer()
    {
        // TODO: check what will happen if close a uninitailized socket.
        close(tcp_data_.server_sockfd);
        close(tcp_data_.connected_sockfd);
    }

    bool send_to_peer(const MessageType &message)
    {
        // TODO: direct serialize to send buffer
        char buffer[message::size_of_message<MessageType>()];
        message::serialize<MessageType>(message, buffer);

        return send_buffer_.write_to_buff_and_trige_send(buffer, tcp_data_.connected_sockfd);
    }

    // actual recv is handled by a background thread.
    bool recv_from_peer(MessageType &message)
    {
        char buffer[message::size_of_message<MessageType>()];
        bool status = recv_buffer_.read(buffer);
        if(status == false)
        {
            return false;
        }
        else
        {
            // TODO: deserialize on the fly
            message::deserialize<MessageType>(buffer, message);
            return false;
        }
    }

    TcpRecvBuffer<message::size_of_message<MessageType>()> recv_buffer_;
    TcpSentBuffer<message::size_of_message<MessageType>()> send_buffer_;

    TcpData tcp_data_;
    TcpConfig config_;
};
}