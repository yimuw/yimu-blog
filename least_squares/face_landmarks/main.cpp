#include <iostream>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "face_alignment.h"

void gt_test(cv::Mat& image1, cv::Mat& image2, std::vector<Landmark>&)
{
    image1 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/test-im.jpg");
    assert(image1.rows > 0 && "fail to read image");
    cv::Mat transfrom = (cv::Mat_<float>(2, 3) << 1., 0.0, 3, -0.1, 1.0, 40.);
    cv::warpAffine(image1, image2, transfrom, image1.size(), cv::WARP_INVERSE_MAP);
}

void read_test(cv::Mat& image1, cv::Mat& image2, std::vector<Landmark>& landmarks)
{
    image2 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/yimu-base.jpg");
    image1 = cv::imread("/home/yimu/Desktop/blog/yimu-blog/least_squares/face_landmarks/yimu2.jpg");
    assert(image1.rows > 0 && "fail to read image");
    assert(image2.rows > 0 && "fail to read image");

    landmarks = {
        { 181, 258 },
        { 294, 261 },
        { 198, 334 },
        { 236, 340 },
        { 273, 339 },
        { 188, 387 },
        { 235, 388 },
        { 286, 389 },
        { 112, 269 },
        { 114, 337 },
        { 137, 405 },
        { 237, 469 },
        { 322, 422 },
        { 358, 347 },
        { 368, 282 }
    };
}

int main(int argc, char* argv[])
{
    std::vector<Landmark> landmarks;
    cv::Mat im_src, im_des;
    read_test(im_src, im_des, landmarks);

    FaceAlignment a;
    a.estimation_transformation(im_src, im_des, landmarks);

    return 0;
}
