#include "comms.h"
#include "utils.h"

int main()
{
    MutipleReaderQueue queue_read;
    queue_read.q_shared_ = allocate_shmem("shared_memory");

    double data_readed1[test_len];
    size_t count = 0;

    DataBlob* d = new DataBlob;
    std::cout << "start reading..." << std::endl;
    while (count < test_len) {
        // TODO: Large stack varible doesn't work in multi-thread env??????
        while (queue_read.read(*d)) {
            data_readed1[count] = d->check_sum;
            std::cout << "read: " << d->check_sum << std::endl;
            count++;
        }
    }

    for (size_t i = 0; i < test_len; ++i) {
        std::cout << "info_readed[i]: " << data_readed1[i] << std::endl;
        assert(data_readed1[i] == i + 1);
    }
    std::cout << "pass test" << std::endl;

    return 0;
}
