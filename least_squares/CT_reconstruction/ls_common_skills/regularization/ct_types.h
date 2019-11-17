#pragma once

#include <opencv2/opencv.hpp>


// A.T A x = A.T b
struct NormalEquation
{    
    NormalEquation() = default;

    NormalEquation(const size_t num_variables)
    {
        lhs = cv::Mat(num_variables, num_variables, CV_32F, cv::Scalar(0.));
        rhs = cv::Mat(num_variables, 1, CV_32F, cv::Scalar(0.));
    }

    cv::Mat lhs;
    cv::Mat rhs;
};