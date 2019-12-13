#pragma once

#include <cstdint>

namespace comms {

// Simple circular buffer protected by mutex
template <uint32_t BufferLength, uint32_t CellSizeByte>
class CircularBuffer {
public:
    using Byte = char;
    struct Cell {
        // a mutex to protect each cell for better concurrency.
        std::mutex mtx;
        Byte blob[CellSizeByte];
    };

    bool has_data()
    {
        std::lock_guard<std::mutex> lck(index_mtx_);
        assert(write_idx_ >= read_idx_);
        return write_idx_ != read_idx_;
    }

    bool write(char const* const data_ptr)
    {
        const size_t write_idx_warp = write_idx_ % BufferLength;
        {
            std::lock_guard<std::mutex> lck(buffer_.at(write_idx_warp).mtx);
            memcpy(buffer_[write_idx_warp].blob, data_ptr, CellSizeByte);
        }

        {
            std::lock_guard<std::mutex> lck(index_mtx_);
            ++write_idx_;

            if (write_idx_ - read_idx_ > BufferLength) {
                read_idx_ = write_idx_ - BufferLength;
            }
        }
        return true;
    }

    // it is read and operation on read data.
    template <typename ProcessFunction>
    bool process(ProcessFunction& process_function)
    {
        if (read_idx_ == write_idx_) {
            return false;
        } else {
            bool status = false;
            {
                std::lock_guard<std::mutex> lck(buffer_.at(read_idx_ % BufferLength).mtx);
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

    size_t read_idx_{ 0 };
    size_t write_idx_{ 0 };

    std::vector<Cell> buffer_ = std::vector<Cell>(BufferLength);
};
}