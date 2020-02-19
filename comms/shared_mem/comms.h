#pragma once

#include <algorithm> // std::sort
#include <assert.h>
#include <atomic>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <vector>

class Spinlock {
public:
    std::atomic_flag flag_{ ATOMIC_FLAG_INIT };

    void lock()
    {
        while (!try_lock()) {
            usleep(speed_time_us);
        }
    }

    inline bool try_lock()
    {
        return !flag_.test_and_set();
    }

    void unlock()
    {
        flag_.clear();
    }

    // config
    uint32_t speed_time_us = 50;
};

using Lock = Spinlock;
// #include <mutex>
// using Lock = std::mutex;

class lock_guard {
public:
    Lock& l_;

    lock_guard(Lock& l)
        : l_(l)
    {
        l_.lock();
    }

    ~lock_guard()
    {
        l_.unlock();
    }
};

class Semaphore {
public:
    std::atomic<int32_t> count_;

    inline void increase()
    {
        count_++;
    }

    inline void decrease()
    {
        count_--;
    }

    inline bool is_zero()
    {
        int32_t expect = 0;
        return count_.compare_exchange_strong(expect, 0);
    }
};

constexpr size_t RAW_DATA_CHAR_SIZE = 1024 * 1024 * 10;

struct DataBlob {
    double check_sum = -1;
    char raw_data[RAW_DATA_CHAR_SIZE];
};

struct Cell {
    Lock writer_lock_;
    Semaphore smf_;

    DataBlob data;
};

constexpr int32_t QUEUE_LEN = 10;

struct MutipleReaderQueueShared {
    bool try_write(const DataBlob& d)
    {
        Cell& m = messages_[write_idx_ % QUEUE_LEN];

        // If some processes is reading, do nothing
        if (!m.smf_.is_zero()) {
            return false;
        }

        {
            // The write operation is protected by a lock *in the message*.
            lock_guard lock(m.writer_lock_);
            m.data = d;

            {
                // Index operation is protected by lock for *write* and *read*.
                // Because index operation is light,
                // using a lock doesn't hurt the performance much.
                lock_guard lock(operation_lock_);
                ++write_idx_;

                // Queue warp around.
                if (write_idx_ - farthest_read_idx_ > QUEUE_LEN) {
                    int32_t last_farest_read_idx = farthest_read_idx_;
                    farthest_read_idx_++;

                    assert(farthest_read_idx_ = write_idx_ - QUEUE_LEN);
                    // doesn't hold when int overflow
                    assert(farthest_read_idx_ > last_farest_read_idx);
                }
            }
        }

        return true;
    }

    bool write(const DataBlob& d)
    {
        while (!try_write(d)) {
            // Unlikely. Happends when the queue warp around.
            std::cout << "write|spinning" << std::endl;
            usleep(1000);
        }

        return true;
    }

    std::atomic<int32_t> write_idx_{ 0 };
    std::atomic<int32_t> farthest_read_idx_{ 0 };

    Lock operation_lock_;
    Cell messages_[QUEUE_LEN];
};

struct MutipleReaderQueue {
    bool try_read(DataBlob& d)
    {
        Cell* m_ptr;
        {
            // make sure index operation is locked.
            // Index operation is light-weighted.
            // Using a lock doesn't hurt performance much.
            lock_guard lock(q_shared_->operation_lock_);

            if (read_idx >= q_shared_->write_idx_) {
                return false;
            }

            // Jump to lastest message.
            // It is the flexibility of shared memory. In TCP, you can't do this.
            if (read_idx < q_shared_->farthest_read_idx_) {
                const int32_t last_read_idx = read_idx;
                read_idx = q_shared_->farthest_read_idx_.load();
                // It doesn't hold when int overflow
                assert(last_read_idx < read_idx);
            }

            m_ptr = &q_shared_->messages_[read_idx % QUEUE_LEN];

            // Check if someone is writing to this cell
            // This only happen if the queue warp around.
            if (!m_ptr->writer_lock_.try_lock()) {
                std::cout << "try_read|someone is writing" << std::endl;
                std::cout << "try_read|m.test_num :" << m_ptr->data.check_sum << std::endl;
                return false;
            } else {
                // Unlock it since I lock the cell in if statement.
                m_ptr->writer_lock_.unlock();
            }

            ++read_idx;
        }

        // Using a semaphore to track how many process is reading the current message.
        // If smf_ > 0, writer shouldn't write anything to it (only when the queue is full).
        m_ptr->smf_.increase();

        d = m_ptr->data;

        // TODO: what if program crashes here? RAII ?
        m_ptr->smf_.decrease();

        return true;
    }

    bool read(DataBlob& d)
    {
        return try_read(d);
    }

    bool write(const DataBlob& d)
    {
        q_shared_->write(d);
        return true;
    }

    // The read idx for this reader.
    int32_t read_idx{ 0 };

    // No ownership, just a plain old pointer.
    // User is responsible to assign the pointer.
    MutipleReaderQueueShared* q_shared_;
};
