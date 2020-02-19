#include "comms.h"
#include "utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << "usage: writer [path-to:yimu-blog/data/image_seqence_basketball]" << std::endl;
        return 0;
    }
    std::string image_dir_path = argv[1];

    SharedMemoryGuard shared_mem("shared_memory");
    MutipleReaderQueue queue_write;

    queue_write.q_shared_ = shared_mem.shm_ptr_;

    namedWindow("write process", cv::WINDOW_AUTOSIZE);

    DataBlob* d = new DataBlob;
    for (size_t i = 0; i < 725; ++i) {
        char filename[9];
        sprintf(filename, "%04zu.jpg", i + 1);
        const std::string path = image_dir_path + "/" + std::string(filename);
        std::cout << "loading... :" << path << std::endl;

        const cv::Mat image = cv::imread(path);
        imshow("write process", image);
        cv::waitKey(1);

        const std::vector<char> smat = serialize_cvmat(image);
        std::copy(smat.begin(), smat.end(), d->raw_data);

        d->check_sum = i + 1;
        queue_write.write(*d);
        usleep(1000 * 20);
    }
    std::cout << "sleeping... keep shmem for 1 sec" << std::endl;
    usleep(1000 * 1000 * 1);

    return 0;
}
