#pragma once

#include <vector>
#include <mutex>   
#include <assert.h>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <unistd.h>
#include <cstring>
#include <atomic>

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

namespace comms
{
// C programmers are crazy...
using Socket = int;

#define PRINT_NAME_VAR(x) std::cout << #x << " :" << x << std::endl;

namespace internal
{
struct SilentCoutInternal
{
    SilentCoutInternal()
    {
        oldCoutStreamBuf = std::cout.rdbuf();
        std::cout.rdbuf( strCout.rdbuf() );
    }

    ~SilentCoutInternal()
    {
        std::cout.rdbuf(oldCoutStreamBuf);
    }

    std::ostringstream strCout;
    std::streambuf* oldCoutStreamBuf = nullptr;
};
}

// TODO: logger
#define SLIENT_COUT_CURRENT_SCOPE    internal::SilentCoutInternal var_name_duplicated;
// #define SLIENT_COUT_CURRENT_SCOPE

template<typename T>
char * cast_to_char_ptr(T * const ptr)
{
    return reinterpret_cast<char *>(ptr);
}

// TODO: class
namespace control
{
std::atomic<bool> quit(false);    // signal flag

void got_signal(int)
{
    quit.store(true);
}

void set_gracefully_exit()
{
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
}

bool program_exit()
{
    return quit.load();
}
}

// Handing ICP data packging 
// e.g. server: send(100byte), send(100byte)
//      client: 20byte = recv(), 110byte = recv(), 70byte = recv
namespace package_sync
{
char control_string[] = "I am just some magic number. like 123456789. \
                         I use it to singal the start of a message.   \
                         make sure you dont type it in your message!  \
                         I am curious how do people handle it in      \
                         pratice? Specitail character? definitely     \
                         not a super long string like this.";

bool send_control_package(int socket)
{
    int n = send(socket, control_string, sizeof(control_string), 0);
    if (n == -1) 
    { 
        std::cout << "send_control_package fail" << std::endl;
        return false;
    }

    return true;
}

enum class SyncStatus
{
    success,
    failure,
    timeout
};

SyncStatus wait_for_control_packge(int socket, char *buf, int &received_data)
{
    // TODO: doesn't work if TCP decide to break control message into 2 receive.
    //       or mutiple control pkg beem grouped into the same tcp recv
    constexpr int SMALL_BUFFER_SIZE  = 1024;
    char small_buf[SMALL_BUFFER_SIZE];

    for(size_t num_try = 0; num_try < 1000; ++num_try)
    {
        int n = recv(socket, small_buf, SMALL_BUFFER_SIZE, 0);
        if (n == -1) 
        { 
            std::cout << "wait_for_control_packge recv fail, retry" << std::endl;
            return SyncStatus::failure;
        }
        if(n == 0)
        {
            std::cout << "wait_for_control_packge server disconnected" << std::endl;
            return SyncStatus::failure;
        }
        
        char * control_string_ptr = strstr(small_buf, control_string);
        if(control_string_ptr == nullptr)
        {
            continue;
        }
        else
        {
            // strip control_string, copy other data to main buffer
            // assume buff is larger than SMALL_BUFFER_SIZE
            const char * mesage_start = control_string_ptr + sizeof(control_string);
            const char * message_end = small_buf + n;
            const int message_size = message_end - mesage_start;
            // assume buf is long enough.
            memcpy( buf, 
                    mesage_start, 
                    message_size);
            received_data = message_size;
            
            return SyncStatus::success;
        }
    }
    std::cout << "wait_for_control_packge timeout" << std::endl;
    return SyncStatus::timeout;
}

}

// https://beej.us/guide/bgnet/html//index.html
bool sendall(int socket, char *buf, int len)
{
    SLIENT_COUT_CURRENT_SCOPE;

    int sent = 0;
    int bytesleft = len;
    int n = -1;

    const int max_tries = 20;
    int num_try = 0;

    while(sent < len) 
    {
        if(num_try++ > max_tries)
        {
            std::cout << "send all time out" << std::endl;
            return false;
        }

        n = send(socket, buf+sent, bytesleft, 0);
        if (n <= 0) 
        { 
            std::cout << "send fail, retry" << std::endl;

            continue; 
        }

        sent += n;
        bytesleft -= n;
        PRINT_NAME_VAR(n);
    }
    PRINT_NAME_VAR(len);
    PRINT_NAME_VAR(sent);
    assert(len == sent);
    
    return len == sent;
}

// https://beej.us/guide/bgnet/html//index.html
bool recv_all(int socket, char *buf, int want_size_byte)
{
    SLIENT_COUT_CURRENT_SCOPE;

    int total = 0;
    int n = -1;
    int want = want_size_byte;

    while(total < want_size_byte) 
    {
        n = recv(socket, buf + total, want - total, 0);
        if (n == -1) 
        { 
            std::cout << "recv fail" << std::endl;
            return false;
        }
        if(n == 0)
        {
            std::cout << "server disconnect" << std::endl;
            return false;
        }
        total += n;
        std::cout << "recv n bytes: " << n << std::endl; 
    }

    std::cout << "total :" << total << std::endl;

    assert(want_size_byte == total && "len != total");
    
    return true;
}

void sigchld_handler(int s)
{
	(void)s; // quiet unused variable warning

	// waitpid() might overwrite errno, so we save and restore it:
	int saved_errno = errno;

	while(waitpid(-1, NULL, WNOHANG) > 0);

	errno = saved_errno;
}


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


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


// Simple circular buffer protected by mutex
template<size_t BufferLength, size_t CellSizeByte>
class CircularBuffer
{
public:
    using Byte = char;
    struct Cell
    {
        // a mutex to protect each cell
        std::mutex mtx;
        Byte blob[CellSizeByte];
    };

    bool has_data()
    {
        std::lock_guard<std::mutex> lck(index_mtx_);
        assert(write_idx_ >= read_idx_);
        return write_idx_ != read_idx_;
    }

    bool write(char const * const data_ptr)
    {
        const size_t write_idx_warp = write_idx_ % BufferLength;
        {
            std::lock_guard<std::mutex> lck (buffer_.at(write_idx_warp).mtx);
            memcpy(buffer_[write_idx_warp].blob, data_ptr, CellSizeByte);
        }

        {
            std::lock_guard<std::mutex> lck(index_mtx_);
            ++write_idx_;

            if(write_idx_ - read_idx_ > BufferLength)
            {
                read_idx_ = write_idx_ - BufferLength;
            }
        }
        return true;
    }

    template<typename ProcessFunction>
    bool process(ProcessFunction &process_function)
    {
        if(read_idx_ == write_idx_)
        {
            return false;
        }
        else
        {
            bool status = false;
            {
                std::lock_guard<std::mutex> lck (buffer_.at(read_idx_ % BufferLength).mtx);
                status = process_function(buffer_[read_idx_ % BufferLength].blob);
            }
            
            {
                std::lock_guard<std::mutex> lck(index_mtx_);
                ++read_idx_;
            }
            return status;
        }
    }

private:

    // seperate index mutex and data mutex for efficiency
    std::mutex index_mtx_;

    size_t read_idx_ {0};
    size_t write_idx_ {0};

    std::vector<Cell> buffer_ = std::vector<Cell>(BufferLength);
};


namespace message
{

template<typename T>   // primary template
constexpr size_t size_of_message()
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));
    return 0;
}

template<typename T>
void serialize(const T &obj, char * const buffer)
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));
}

template<typename T>
void deserialize(char const * const buffer, T &obj)
{
    assert(false && strcat("template specification not implement for",typeid(T).name()));  
}

}

}
