#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dehaze.h"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << "usage: dehaze <path in image>" << std::endl;
    }
    std::string im_path = argv[1];
    std::cout << "reading image from: " << im_path << std::endl;

    cv::Mat haze_im = cv::imread(im_path);
    haze_im.convertTo(haze_im, CV_32FC3, 1 / 255.0);

    Dehaze dehaze;

    dehaze.dehaze(haze_im);

    return 0;
}