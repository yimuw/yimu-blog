#include <vector>
#include <string.h>
#include <mutex>   
#include <assert.h>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <unistd.h>
#include <cstring>
#include <atomic>

namespace comms
{

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
}
