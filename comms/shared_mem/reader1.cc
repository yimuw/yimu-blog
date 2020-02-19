#include "comms.h"
#include "utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using time_point = std::chrono::steady_clock::time_point;

int main()
{
    MutipleReaderQueue queue_read;
    queue_read.q_shared_ = allocate_shmem("shared_memory");

    DataBlob* d = new DataBlob;
    std::cout << "start reading..." << std::endl;

    time_point begin_time = std::chrono::steady_clock::now();
    time_point cur_time = begin_time;

    while (true) {
        cur_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(cur_time - begin_time).count() > 5) {
            std::cout << "time out, exit" << std::endl;
            return 0;
        }

        while (queue_read.read(*d)) {
            cv::Mat mat = deserialize_cvmat(d->raw_data);

            cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);

            cv::imshow("process 1: gray", mat);
            cv::waitKey(1);

            begin_time = std::chrono::steady_clock::now();
        }

        usleep(1000 * 30);
    }

    return 0;
}
