#pragma once

#include <opencv2/opencv.hpp>


class SoftMatting
{

public:

    SoftMatting(const cv::Mat &target_im, const cv::Mat &t_prior)
    {
        // cv::cvtColor(target_im, im_, cv::COLOR_BGR2GRAY);
        im_ = target_im.clone();
        t_prior_ = t_prior.clone();

        cv::imshow("im_", im_);
        cv::imshow("t_prior_", t_prior_);
        // cv::waitKey();

        const int rows = t_prior.rows;
        const int cols = t_prior.cols;

        // float mean_a = cv::sum(t_prior_)[0] / cv::sum(im_)[0];
        A_ = cv::Mat(rows, cols, CV_32FC3, 0.);
        // A_ = t_prior_ / im_;
        grad_A_ = cv::Mat(rows, cols, CV_32FC3, 0.);

        B_ = cv::Mat(rows, cols, CV_32FC1, 0.);
        grad_B_ = cv::Mat(rows, cols, CV_32FC1, 0.);

        T_ = t_prior_.clone();
        // float mean_t  = cv::sum(t_prior_)[0] / (rows * cols);
        //T_ = cv::Mat(rows, cols, CV_32FC1, mean_t);
        grad_T_ = cv::Mat(rows, cols, CV_32FC1, 0.);
    }

    // goal: 1. t = a * target + b
    //       2. t close to t_prior
    // cost:  sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
    //         + weight * sum{i: pixel_index } (t_i - t_prior_i)^2
    void compute_gradient()
    {
        grad_A_.setTo(cv::Vec3f(0., 0., 0.));
        grad_B_.setTo(cv::Scalar(0.));
        grad_T_.setTo(cv::Scalar(0.));

        const int rows = t_prior_.rows;
        const int cols = t_prior_.cols;

        const int half_window_size = 5;
        const int window_stride = 1;

        // sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
        // patches
        for(int row_idx = half_window_size; row_idx < rows - half_window_size; row_idx+=window_stride)
        {
            for(int col_idx = half_window_size; col_idx < cols - half_window_size; col_idx+=window_stride)
            {
                const cv::Point2i j_idx(col_idx, row_idx);
                const cv::Vec3f a_j = A_.at<cv::Vec3f>(j_idx);
                const float b_j = B_.at<float>(j_idx);

                // pixels in a patch
                for(int dy = -half_window_size; dy <= half_window_size; ++dy)
                {
                    for(int dx = -half_window_size; dx <= half_window_size; ++dx)
                    {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);

                        const float t_i = T_.at<float>(i_idx);
                        const cv::Vec3f im_i = im_.at<cv::Vec3f>(i_idx);

                        // For clarity, I keep all the intermidate steps.
                        // let (t_i - a_j * im_i - b_j) = k
                        const float dcost_dk = 2 * (t_i - vec3f_dot(a_j, im_i) - b_j);
                        // grad of k w.r.t variables
                        const float dk_dti = 1.;
                        const cv::Vec3f dk_daj = - im_i;
                        const float dk_dbj = -1.;

                        // const float weight = 1./((half_window_size * 2 + 1) * (half_window_size * 2 + 1));
                        const float weight = 0.1;
                        grad_T_.at<float>(i_idx) += weight * dcost_dk * dk_dti;
                        grad_A_.at<cv::Vec3f>(j_idx) += weight * dcost_dk * dk_daj;
                        grad_B_.at<float>(j_idx) += weight * dcost_dk * dk_dbj;

                    }
                }
            }
        }

        // prior cost: weight * sum{i: pixel_index } (t_i - t_prior_i)^2
        for(int row_idx = 0; row_idx < rows; ++row_idx)
        {
            for(int col_idx = 0; col_idx < cols; ++ col_idx)
            {
                const cv::Point2i pidx(col_idx, row_idx);
                const float t_i = T_.at<float>(pidx);
                const float t_prior_i = t_prior_.at<float>(pidx);
                grad_T_.at<float>(pidx) += prior_weight_ * 2 * (t_i - t_prior_i) * 1.;
            }
        }
    }

    void gradient_descent()
    {
        A_ -= gd_step * grad_A_;
        B_ -= gd_step * grad_B_;
        T_ -= gd_step * grad_T_;
    }

    float vec3f_dot(const cv::Vec3f &a, const cv::Vec3f &b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; 
    }

    cv::Mat run()
    {
        for(int iter = 0; iter < max_iter_; ++iter)
        {
            compute_gradient();
            gradient_descent();

            if(iter % 20 == 0)
            {
                std::cout << "iter: " << iter << std::endl;
                cv::imshow("matting", T_);
                cv::waitKey(1);
            }
        }

        return T_;
    }
    
    // data
    cv::Mat im_;
    cv::Mat t_prior_;

    // variables and its grad
    cv::Mat A_;
    cv::Mat grad_A_;
    cv::Mat B_;
    cv::Mat grad_B_;
    cv::Mat T_;
    cv::Mat grad_T_;

    // config
    int max_iter_ = 50000;
    float prior_weight_ = 0.3;
    float gd_step = 0.01;
};
