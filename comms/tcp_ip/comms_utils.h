#include <vector>
#include <string.h>
#include <mutex>   
#include <assert.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

namespace comms
{

#define PRINT_NAME_VAR(x) std::cout << #x << " :" << x << std::endl;

namespace control
{
std::atomic<bool> quit(false);    // signal flag

void got_signal(int)
{
    quit.store(true);
}

void gracefully_exit()
{
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
}

bool problem_exit()
{
    return quit.load();
}
}

namespace package_sync
{
char control_string[] = "Yo, I am control packge";

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

bool wait_for_control_packge(int socket, char *buf, int buf_size)
{
    for(size_t num_try = 0; num_try < 100;)
    {
        int n = recv(socket, buf, buf_size, 0);
        if (n == -1) 
        { 
            std::cout << "wait_for_control_packge recv fail, retry" << std::endl;
            usleep(50);
            continue; 
        }
        if(n == 0)
        {
            std::cout << "wait_for_control_packge server disconnected" << std::endl;
            return false;
        }
        
        if(strcmp(buf, control_string) == 0)
        {
            std::cout << "get control pkg" << std::endl;
            return true;
        }
        else
        {
            std::cout << "ignore other pkg" << std::endl;
        }

        usleep(50);
    }
    return false;
}
}

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

// #include <opencv2/core/core.hpp>

// // https://stackoverflow.com/questions/4170745/serializing-opencv-mat-vec3f/21444792#21444792
// std::vector<char> serialize_cvmat(const cv::Mat& mat)
// {
// 	std::vector<char> serialized_data;
// 	// 4 int32_t
// 	serialized_data.resize(4 * 3);

//     int cols, rows, type;
//     bool continuous;

//     cols = mat.cols; 
//     rows = mat.rows; 
//     type = mat.type();
//     continuous = mat.isContinuous();

//     assert(continuous == true);

//     *reinterpret_cast<uint32_t *>(&serialized_data.at(0)) = cols;
//     *reinterpret_cast<uint32_t *>(&serialized_data.at(4)) = rows;
//     *reinterpret_cast<uint32_t *>(&serialized_data.at(8)) = type;
//     // mat.create(rows, cols, type);

//     const size_t data_size = rows * cols * mat.elemSize();
//     serialized_data.insert(serialized_data.end(), mat.ptr(), mat.ptr() + data_size);

// 	return serialized_data; 
// }


// cv::Mat deserialize_cvmat(const char * const serialized_data)
// {
//     int cols, rows, type;
//     cols = *reinterpret_cast<const uint32_t *>(&serialized_data[0]); 
//     rows = *reinterpret_cast<const uint32_t *>(&serialized_data[4]);
//     type = *reinterpret_cast<const uint32_t *>(&serialized_data[8]);

//     cv::Mat res_mat;
//     res_mat.create(rows, cols, type);

//     const size_t data_size = rows * cols * res_mat.elemSize();

//     auto imdata_start = serialized_data + 4 * 3;
//     auto imdata_end = serialized_data + 4 * 3 + data_size;

//     std::copy(imdata_start, imdata_end, res_mat.ptr());

// 	return res_mat; 
// }