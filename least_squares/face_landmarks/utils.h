#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#define PRINT_NAME_VAR(var) std::cout << #var << " :" << var << std::endl;

cv::Point2f apply_affine(const cv::Mat& trans, const cv::Point2f& p)
{
    assert(trans.rows == 6);
    assert(trans.cols == 1);

    const float a = trans.at<float>(0, 0);
    const float b = trans.at<float>(1, 0);
    const float e = trans.at<float>(2, 0);
    const float c = trans.at<float>(3, 0);
    const float d = trans.at<float>(4, 0);
    const float f = trans.at<float>(5, 0);

    cv::Point2f p_trans;
    p_trans.x = a * p.x + b * p.y + e;
    p_trans.y = c * p.x + d * p.y + f;
    return p_trans;
}

cv::Mat inverse_affine(const cv::Mat& trans)
{
    assert(trans.rows == 6);
    assert(trans.cols == 1);

    const float a = trans.at<float>(0, 0);
    const float b = trans.at<float>(1, 0);
    const float e = trans.at<float>(2, 0);
    const float c = trans.at<float>(3, 0);
    const float d = trans.at<float>(4, 0);
    const float f = trans.at<float>(5, 0);

    cv::Mat A = (cv::Mat_<float>(2, 2) << a + 1e-8, b, c + 1e-8, d);
    cv::Mat bb = (cv::Mat_<float>(2, 1) << e, f);
    cv::Mat A_inv = A.inv();
    cv::Mat b_inv = -A_inv * bb;

    cv::Mat res = (cv::Mat_<float>(6, 1)
            << A_inv.at<float>(0, 0),
        A_inv.at<float>(0, 1),
        b_inv.at<float>(0, 0),
        A_inv.at<float>(1, 0),
        A_inv.at<float>(1, 1),
        b_inv.at<float>(1, 0));
    return res;
}

void warp(const cv::Mat& im,
    cv::Mat& res,
    const cv::Mat& affine_trans,
    const int x_start,
    const int x_len,
    const int y_start,
    const int y_len)
{
    cv::Mat transfrom = (cv::Mat_<float>(2, 3)
            << affine_trans.at<float>(0, 0),
        affine_trans.at<float>(1, 0),
        affine_trans.at<float>(2, 0),
        affine_trans.at<float>(3, 0),
        affine_trans.at<float>(4, 0),
        affine_trans.at<float>(5, 0));

    // Not efficient
    cv::Mat warped_whole;
    cv::warpAffine(im, warped_whole, transfrom, im.size(), cv::WARP_INVERSE_MAP);

    for (float y = y_start; y < y_start + y_len; ++y) {
        for (float x = x_start; x < x_start + x_len; ++x) {
            res.at<float>(y, x) = warped_whole.at<float>(y, x);
        }
    }
}