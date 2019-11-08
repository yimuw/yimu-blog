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

// template<size_t CellSizeByte>
class TcpPeer
{
public:
    ~TcpPeer()
    {
        // TODO: check what will happen if close a uninitailized socket.
        close(tcp_data_.server_sockfd);
        close(tcp_data_.connected_sockfd);
    }

    bool send_to_peer(char const * const data_ptr)
    {
        return send_buffer_.write_to_buff_and_trige_send(data_ptr, tcp_data_.connected_sockfd);
    }

    // actual recv is handled by a background thread.
    bool recv_from_peer(char * const target_data_ptr)
    {
        return recv_buffer_.read(target_data_ptr);
    }

    TcpRecvBuffer<sizeof(int)> recv_buffer_;
    TcpSentBuffer<sizeof(int)> send_buffer_;

    TcpData tcp_data_;
    TcpConfig config_;
};
}