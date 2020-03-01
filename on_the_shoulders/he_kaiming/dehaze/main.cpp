#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dehaze.h"


int main(int argc, char* argv[])
{
    cv::Mat haze_im = cv::imread("/home/yimu/Desktop/blog/yimu-blog/on_the_shoulders/he_kaiming/dehaze/tiananmen1.png");
    haze_im.convertTo(haze_im, CV_32FC3, 1/255.0);

    Dehaze dehaze;

    // cv::imshow("m_pad", haze_im);
    // cv::waitKey(0);

    dehaze.dehaze(haze_im);

    return 0;
}