#include <iostream>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "face_alignment.h"

int main(int argc, char* argv[])
{
    const cv::Mat image1 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/0001.jpg");
    cv::Mat image2;
    cv::Mat transfrom = (cv::Mat_<float>(2,3) << 1., 0.1, 3, -0.1, 1.05, 0.);
    cv::warpAffine( image1, image2, transfrom, image1.size(), cv::WARP_INVERSE_MAP);

    // cv::imshow("im1", image);  
    // cv::imshow("trans", image2);
    // cv::waitKey(0);
    
    FaceAlignment a;
    a.estimation_transformation(image1, image2);

    return 0;
}
