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


namespace comms
{

template<size_t CellSizeByte>
class TcpRecvBuffer
{
public:
    ~TcpRecvBuffer()
    {
        if(recv_thread_.joinable())
        {
            std::cout << "joining recv_thread_" << std::endl;
            recv_thread_.join();
        }
    }

    bool read(char * const target_data_ptr)
    {
        if(has_data() == false)
        {
            return false;
        }

        if(recv_thread_alive_ == false)
        {
            return false;
        }

        auto copy_to_buffer = [&target_data_ptr](char const * const src_data_ptr)
        {
            memcpy(target_data_ptr, src_data_ptr, CellSizeByte);
            return true;
        };

        const bool status = buffer_.process(copy_to_buffer);
        return status;
    }

    // recv is handled by a background thread.
    void start_recv_thread(Socket connected_socket)
    {
        recv_thread_ = std::thread(
            [this, connected_socket]()
            {
                recv_thread_alive_ = true;
                std::cout << "recv buffer, thread started" << std::endl;
                this->recv_data_loop(connected_socket);
                recv_thread_alive_ = false;
            }
        );
    }

private:
    bool has_data()
    {
        return buffer_.has_data();
    }

    void recv_data_loop(Socket connected_socket)
    {
        while(control::program_exit() == false)
        {
            bool status = recv_data_blob_and_write_to_queue(connected_socket);
            if(status == false)
            {
                std::cout << "recv data thread quit" << std::endl;
                break;
            }
            else
            {
                std::cout << "background thread receive data" << std::endl;
            }
            
        }
    }

    // Blocking call
    // TODO: if sender crash in the middle of sending, the function is blocked forever.
    bool recv_data_blob_and_write_to_queue(Socket connected_socket)
    {
        // TODO: super large array in stack is not good.
        char buf[CellSizeByte];
        int received_data = 0;
        namespace sync = comms::package_sync;
        sync::SyncStatus status = sync::wait_for_control_packge(connected_socket, buf, received_data);
        if(status == sync::SyncStatus::success)
        {
            const int rest_bytes = CellSizeByte - received_data;
            assert(rest_bytes >= 0);
            if(rest_bytes > 0)
            {
                bool recv_status = recv_all(connected_socket, buf + received_data, 
                    CellSizeByte - received_data);
                if (recv_status == false)
                {
                    perror("recv_all failed");
                    std::cerr << "recv_all failed" << std::endl;
                    return false;
                }
            }

            // TODO: not efficient. Do one copy.
            buffer_.write(buf);
            return true;
        }
        else
        {
            std::cout << "tcp recv fail" << std::endl;
            return false;
        }
    }

private:

    static constexpr size_t BUFFER_LENGTH {10};
    CircularBuffer<BUFFER_LENGTH, CellSizeByte> buffer_;

    std::thread recv_thread_;
    std::atomic_bool recv_thread_alive_ {false};
};
}