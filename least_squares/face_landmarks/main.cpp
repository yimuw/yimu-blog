#include <iostream>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "face_alignment.h"

void gt_test(cv::Mat& image1, cv::Mat& image2)
{
    image1 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/test-im.jpg");
    assert(image1.rows > 0 && "fail to read image");
    cv::Mat transfrom = (cv::Mat_<float>(2, 3) << 1., 0.0, 3, -0.1, 1.0, 40.);
    cv::warpAffine(image1, image2, transfrom, image1.size(), cv::WARP_INVERSE_MAP);
}

void read_test(cv::Mat& image1, cv::Mat& image2)
{
    image2 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/yimu-base.jpg");
    image1 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/yimu2.jpg");
    assert(image1.rows > 0 && "fail to read image");
    assert(image2.rows > 0 && "fail to read image");
}

int main(int argc, char* argv[])
{
    cv::Mat image1, image2;
    read_test(image1, image2);

    FaceAlignment a;
    a.estimation_transformation(image1, image2);

    return 0;
}
