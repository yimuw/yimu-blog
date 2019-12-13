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
#include "buffer.h"


namespace comms
{

template<size_t CellSizeByte>
class TcpSendBuffer
{
public:
    // Copy to buffer, and trige tcp send.
    // Not guarantee to publish successfully. 
    bool write_to_buff_and_trige_send(char const * const data_ptr, const Socket &connected_client)
    {
        // copy data to buffer
        if(send_buffer_.write(data_ptr))
        {
            // Trige a sent
            // alternatively, using a thread and a conditional variable.
            return send_avaliable_data_in_queue_to_client(connected_client);
        }
        else
        {
            std::cout << "write failed!" << std::endl;
            return false;
        } 
    }

    bool send_avaliable_data_in_queue_to_client(const Socket &connected_client)
    {
        auto icp_send_function = [&connected_client](char * const data_ptr)
        {
            package_sync::send_control_package(connected_client);
            if(sendall(connected_client, data_ptr, CellSizeByte) == false)
            {
                return false;
            }

            return true;
        };

        // send everything in buffer
        while(send_buffer_.process(icp_send_function) == true);

        return true;
    }

private:

    static constexpr size_t BUFFER_LENGTH {10};
    CircularBuffer<BUFFER_LENGTH, CellSizeByte> send_buffer_;
};
}