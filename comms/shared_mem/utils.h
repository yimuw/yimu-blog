#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>

template <class T>
T* mmap_shmem(const std::string& shmem_name, std::string& error_msg)
{
    int fd = -1;
    fd = shm_open(shmem_name.c_str(), O_CREAT | O_RDWR, 0666);

    if (fd == -1) {
        error_msg = "open";
        return nullptr;
    }
    if (ftruncate(fd, sizeof(T))) {
        error_msg = "ftruncate";
        close(fd);
        return nullptr;
    }

    T* ret = (T*)mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ret == MAP_FAILED) {
        error_msg = "mmap";
        return nullptr;
    }
    return ret;
}

MutipleReaderQueueShared* allocate_shmem(const std::string& key)
{
    MutipleReaderQueueShared* shm_ptr = nullptr;

    std::string error_msg;
    shm_ptr = mmap_shmem<MutipleReaderQueueShared>(key, error_msg);
    assert(shm_ptr);

    return shm_ptr;
}

class SharedMemoryGuard {
public:
    SharedMemoryGuard(const std::string& key)
    {
        hacky_init_helper = new MutipleReaderQueueShared();

        shm_ptr_ = allocate_shmem(key);

        //hacky init
        memcpy(shm_ptr_, hacky_init_helper, sizeof(MutipleReaderQueueShared));
    }

    ~SharedMemoryGuard()
    {
        memcpy(shm_ptr_, hacky_init_helper, sizeof(MutipleReaderQueueShared));
        munmap(shm_ptr_, sizeof(MutipleReaderQueueShared));

        delete hacky_init_helper;
    }

    MutipleReaderQueueShared* hacky_init_helper;
    MutipleReaderQueueShared* shm_ptr_;
};

const size_t test_len = 100;

// Technically, we don't need serialization. But cv::Mat have some heap data.
// https://stackoverflow.com/questions/4170745/serializing-opencv-mat-vec3f/21444792#21444792
std::vector<char> serialize_cvmat(const cv::Mat& mat)
{
    std::vector<char> serialized_data;
    // 4 int32_t
    serialized_data.resize(4 * 3);

    int cols, rows, type;
    bool continuous;

    cols = mat.cols;
    rows = mat.rows;
    type = mat.type();
    continuous = mat.isContinuous();

    assert(continuous == true);

    *reinterpret_cast<uint32_t*>(&serialized_data.at(0)) = cols;
    *reinterpret_cast<uint32_t*>(&serialized_data.at(4)) = rows;
    *reinterpret_cast<uint32_t*>(&serialized_data.at(8)) = type;
    // mat.create(rows, cols, type);

    const size_t data_size = rows * cols * mat.elemSize();
    serialized_data.insert(serialized_data.end(), mat.ptr(), mat.ptr() + data_size);

    return serialized_data;
}

cv::Mat deserialize_cvmat(const char* const serialized_data)
{

    int cols, rows, type;
    cols = *reinterpret_cast<const uint32_t*>(&serialized_data[0]);
    rows = *reinterpret_cast<const uint32_t*>(&serialized_data[4]);
    type = *reinterpret_cast<const uint32_t*>(&serialized_data[8]);

    cv::Mat res_mat;
    res_mat.create(rows, cols, type);

    const size_t data_size = rows * cols * res_mat.elemSize();

    auto imdata_start = serialized_data + 4 * 3;
    auto imdata_end = serialized_data + 4 * 3 + data_size;

    std::copy(imdata_start, imdata_end, res_mat.ptr());

    return res_mat;
}