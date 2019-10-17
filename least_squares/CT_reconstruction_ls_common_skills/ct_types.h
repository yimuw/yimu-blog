#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


struct ScanLine
{
    std::vector<int> scaned_indexes;
    float accumulatedValue = 0.;
};


struct DenseEquation
{
    DenseEquation(const size_t num_equations, const size_t num_variables)
    {
        A = cv::Mat(num_equations, num_variables, CV_32F, cv::Scalar(0.));
        b = cv::Mat(num_equations, 1, CV_32F, cv::Scalar(0.));
    }

    cv::Mat A;
    cv::Mat b;
};