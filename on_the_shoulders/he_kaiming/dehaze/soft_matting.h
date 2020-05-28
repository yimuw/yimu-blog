#pragma once

#include <opencv2/opencv.hpp>

class SoftMatting {
public:
    virtual cv::Mat run() = 0;

    virtual std::string config_to_string() const = 0;
};

///////////////////////////////////////////////////////////////////////////////
// SoftMattingDirect
///////////////////////////////////////////////////////////////////////////////
class SoftMattingDirect : public SoftMatting {

public:
    SoftMattingDirect(const cv::Mat& target_im, const cv::Mat& t_prior)
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

        const int half_window_size = half_window_size_;
        const int window_stride = 1;

        // sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
        // patches
        for (int row_idx = half_window_size; row_idx < rows - half_window_size; row_idx += window_stride) {
            for (int col_idx = half_window_size; col_idx < cols - half_window_size; col_idx += window_stride) {
                const cv::Point2i j_idx(col_idx, row_idx);
                const cv::Vec3f a_j = A_.at<cv::Vec3f>(j_idx);
                const float b_j = B_.at<float>(j_idx);

                // pixels in a patch
                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);

                        const float t_i = T_.at<float>(i_idx);
                        const cv::Vec3f im_i = im_.at<cv::Vec3f>(i_idx);

                        // For clarity, I keep all the intermidate steps.
                        // let (t_i - a_j * im_i - b_j) = k
                        const float dcost_dk = 2 * (t_i - vec3f_dot(a_j, im_i) - b_j);
                        // grad of k w.r.t variables
                        const float dk_dti = 1.;
                        const cv::Vec3f dk_daj = -im_i;
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
        for (int row_idx = 0; row_idx < rows; ++row_idx) {
            for (int col_idx = 0; col_idx < cols; ++col_idx) {
                const cv::Point2i pidx(col_idx, row_idx);
                const float t_i = T_.at<float>(pidx);
                const float t_prior_i = t_prior_.at<float>(pidx);
                grad_T_.at<float>(pidx) += prior_weight_ * 2 * (t_i - t_prior_i) * 1.;
            }
        }
    }

    void gradient_descent()
    {
        cv::imshow("grad_T_", grad_T_);
        A_ -= gd_step * grad_A_;
        B_ -= gd_step * grad_B_;
        T_ -= gd_step * grad_T_;
    }

    float vec3f_dot(const cv::Vec3f& a, const cv::Vec3f& b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    cv::Mat run() override
    {
        for (int iter = 0; iter < max_iter_; ++iter) {
            compute_gradient();
            gradient_descent();

            if (iter % 20 == 0) {
                std::cout << "iter: " << iter << std::endl;
                cv::imshow("matting", T_);
                cv::waitKey(10);
            }
        }

        return T_;
    }

    std::string config_to_string() const
    {
        return "maxTter:" + std::to_string(max_iter_)
            + " hwinSize:" + std::to_string(half_window_size_)
            + " priorWeight:" + std::to_string(prior_weight_)
            + " gdStep:" + std::to_string(gd_step);
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
    int max_iter_ = 1000;
    int half_window_size_ = 2;
    float prior_weight_ = 0.3;
    float gd_step = 0.01;
};

///////////////////////////////////////////////////////////////////////////////
// SoftMattingEliminated
///////////////////////////////////////////////////////////////////////////////
class SoftMattingEliminated : public SoftMatting {

public:
    SoftMattingEliminated(const cv::Mat& target_im, const cv::Mat& t_prior)
    {
        // cv::cvtColor(target_im, im_, cv::COLOR_BGR2GRAY);
        im_ = target_im.clone();
        t_prior_ = t_prior.clone();

        cv::imshow("im_", im_);
        cv::imshow("t_prior_", t_prior_);
        // cv::waitKey();

        const int rows = t_prior.rows;
        const int cols = t_prior.cols;

        std::cout << "rows: " << rows << " cols:" << cols << std::endl;

        T_ = t_prior_.clone();
        grad_T_ = cv::Mat(rows, cols, CV_32FC1, 0.);
    }

    void compute_G()
    {
        const int rows = t_prior_.rows;
        const int cols = t_prior_.cols;

        G_map_ = std::vector<std::vector<cv::Mat>>(rows, std::vector<cv::Mat>(cols, cv::Mat()));

        const int half_window_size = half_window_size_;
        const int window_size = half_window_size * 2 + 1;
        const int num_elements_in_window = window_size * window_size;
        const int window_stride = 1;

        auto I = cv::Mat::eye(num_elements_in_window, num_elements_in_window, CV_32FC1);

        // sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
        // patches
        for (int row_idx = half_window_size; row_idx < rows - half_window_size; row_idx += window_stride) {
            for (int col_idx = half_window_size; col_idx < cols - half_window_size; col_idx += window_stride) {
                cv::Mat G(num_elements_in_window, 4, CV_32FC1, 0.);

                // pixels in a patch
                int count = 0;
                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);

                        const cv::Vec3f im_i = im_.at<cv::Vec3f>(i_idx);

                        G.at<float>(count, 0) = im_i[0];
                        G.at<float>(count, 1) = im_i[1];
                        G.at<float>(count, 2) = im_i[2];
                        G.at<float>(count, 3) = 1.;

                        count++;
                    }
                }
                assert(count == num_elements_in_window);
                cv::Mat G_bar = I - G * (G.t() * G + 1e-2 * cv::Mat::eye(4, 4, CV_32FC1)).inv() * G.t();
                cv::Mat matting_grad = 2 * G_bar.t() * G_bar;

                G_map_[row_idx][col_idx] = matting_grad;
            }
        }
    }

    // goal: 1. t = a * target + b
    //       2. t close to t_prior
    // cost:  sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
    //         + weight * sum{i: pixel_index } (t_i - t_prior_i)^2
    void compute_gradient()
    {
        grad_T_.setTo(cv::Scalar(0.));

        const int rows = t_prior_.rows;
        const int cols = t_prior_.cols;

        const int half_window_size = half_window_size_;
        const int window_size = half_window_size * 2 + 1;
        const int num_elements_in_window = window_size * window_size;
        const int window_stride = 1;

        auto I = cv::Mat::eye(num_elements_in_window, num_elements_in_window, CV_32FC1);

        // sum_{j:patch idx} sum_{i:pixel idx in patch} (t_i - a_j * im_i - b_j)^2
        // patches
        for (int row_idx = half_window_size; row_idx < rows - half_window_size; row_idx += window_stride) {
            for (int col_idx = half_window_size; col_idx < cols - half_window_size; col_idx += window_stride) {
                cv::Mat t_vec(num_elements_in_window, 1, CV_32FC1, 0.);

                // pixels in a patch
                int count = 0;
                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);
                        t_vec.at<float>(count, 0) = T_.at<float>(i_idx);
                        count++;
                    }
                }
                assert(count == num_elements_in_window);
                cv::Mat matting_grad = G_map_[row_idx][col_idx] * t_vec;

                // pixels in a patch
                count = 0;
                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);

                        grad_T_.at<float>(i_idx) += matting_grad.at<float>(count, 0);
                        count++;
                    }
                }
                assert(count == num_elements_in_window);
            }
        }

        // prior cost: weight * sum{i: pixel_index } (t_i - t_prior_i)^2
        for (int row_idx = 0; row_idx < rows; ++row_idx) {
            for (int col_idx = 0; col_idx < cols; ++col_idx) {
                const cv::Point2i pidx(col_idx, row_idx);
                const float t_i = T_.at<float>(pidx);
                const float t_prior_i = t_prior_.at<float>(pidx);
                grad_T_.at<float>(pidx) += prior_weight_ * 2 * (t_i - t_prior_i);
            }
        }
    }

    void gradient_descent()
    {
        T_ -= gd_step * grad_T_;
    }

    float vec3f_dot(const cv::Vec3f& a, const cv::Vec3f& b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    cv::Mat run() override
    {
        // percompute constant terms
        compute_G();

        for (int iter = 0; iter < max_iter_; ++iter) {
            std::cout << "iter: " << iter << std::endl;
            compute_gradient();
            std::cout << "norm of grad: " << cv::norm(grad_T_) << std::endl;
            gradient_descent();

            if (iter % 1 == 0) {
                cv::imshow("matting", T_);
                cv::imshow("grad_T_", 1e1 * grad_T_);
                cv::waitKey(10);
            }
        }

        return T_;
    }

    std::string config_to_string() const
    {
        return "maxTter:" + std::to_string(max_iter_)
            + " hwinSize:" + std::to_string(half_window_size_)
            + " priorWeight:" + std::to_string(prior_weight_)
            + " gdStep:" + std::to_string(gd_step);
    }

    // data
    cv::Mat im_;
    cv::Mat t_prior_;

    // variables and its grad
    cv::Mat T_;
    cv::Mat grad_T_;

    std::vector<std::vector<cv::Mat>> G_map_;

    // config
    int max_iter_ = 500;
    int half_window_size_ = 2;
    float prior_weight_ = 0.3;
    float gd_step = 0.02;
};

///////////////////////////////////////////////////////////////////////////////
// GuideFilter
///////////////////////////////////////////////////////////////////////////////
class GuideFilter : public SoftMatting {

public:
    GuideFilter(const cv::Mat& target_im, const cv::Mat& t_prior)
    {
        // cv::cvtColor(target_im, im_, cv::COLOR_BGR2GRAY);
        im_ = target_im.clone();
        t_prior_ = t_prior.clone();

        cv::imshow("im_", im_);
        cv::imshow("t_prior_", t_prior_);

        const int rows = t_prior.rows;
        const int cols = t_prior.cols;

        std::cout << "rows: " << rows << " cols:" << cols << std::endl;
    }

    cv::Mat run() override
    {
        const int rows = t_prior_.rows;
        const int cols = t_prior_.cols;

        const int half_window_size = half_window_size_;
        const int window_size = half_window_size * 2 + 1;
        const int num_elements_in_window = window_size * window_size;
        const int window_stride = 1;

        auto I = cv::Mat::eye(num_elements_in_window, num_elements_in_window, CV_32FC1);

        // To handle pixels near the broader.
        cv::Mat ab_counter(rows, cols, CV_32FC1, 0.);

        cv::Mat a(rows, cols, CV_32FC3, 0.);
        cv::Mat b(rows, cols, CV_32FC1, 0.);

        // Compute a&b
        for (int row_idx = half_window_size; row_idx < rows - half_window_size; row_idx += window_stride) {
            for (int col_idx = half_window_size; col_idx < cols - half_window_size; col_idx += window_stride) {
                cv::Mat G(num_elements_in_window, 4, CV_32FC1, 0.);
                cv::Mat t_vec(num_elements_in_window, 1, CV_32FC1, 0.);

                // pixels in a patch
                int count = 0;
                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        // i, j are defined in the cost function
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);

                        const float t_i = t_prior_.at<float>(i_idx);

                        const cv::Vec3f im_i = im_.at<cv::Vec3f>(i_idx);

                        // std::cout << "count: " << count << std::endl;
                        G.at<float>(count, 0) = im_i[0];
                        G.at<float>(count, 1) = im_i[1];
                        G.at<float>(count, 2) = im_i[2];
                        G.at<float>(count, 3) = 1.;

                        t_vec.at<float>(count, 0) = t_i;
                        count++;
                    }
                }
                assert(count == num_elements_in_window);
                cv::Mat ab = (G.t() * G + 1e-2 * cv::Mat::eye(4, 4, CV_32FC1)).inv() * G.t() * t_vec;

                for (int dy = -half_window_size; dy <= half_window_size; ++dy) {
                    for (int dx = -half_window_size; dx <= half_window_size; ++dx) {
                        const cv::Point2i i_idx(col_idx + dx, row_idx + dy);
                        a.at<cv::Vec3f>(i_idx)[0] += ab.at<float>(0, 0);
                        a.at<cv::Vec3f>(i_idx)[1] += ab.at<float>(1, 0);
                        a.at<cv::Vec3f>(i_idx)[2] += ab.at<float>(2, 0);
                        b.at<float>(i_idx) += ab.at<float>(3, 0);
                        ab_counter.at<float>(i_idx) += 1.;
                    }
                }
            }
        }

        cv::Mat filtered(rows, cols, CV_32FC1, 0.);
        // apply the filter
        for (int row_idx = 0; row_idx < rows; ++row_idx) {
            for (int col_idx = 0; col_idx < cols; ++col_idx) {
                const cv::Point2i i_idx(col_idx, row_idx);
                // Compute the mean
                a.at<cv::Vec3f>(i_idx) /= ab_counter.at<float>(i_idx);
                b.at<float>(i_idx) /= ab_counter.at<float>(i_idx);

                filtered.at<float>(i_idx) = im_.at<cv::Vec3f>(i_idx).dot(a.at<cv::Vec3f>(i_idx)) + b.at<float>(i_idx);
            }
        }

        if (true) {
            cv::imshow("filtered", filtered);
            cv::imwrite("filtered.png", 255 * filtered);
            cv::imwrite("t_prior_.png", 255 * t_prior_);
        }

        return filtered;
    }

    std::string config_to_string() const
    {
        return std::string("GuidedFilter") + " hwinSize:" + std::to_string(half_window_size_);
    }

    // data
    cv::Mat im_;
    cv::Mat t_prior_;

    // config

    // should be a function of image size
    int half_window_size_ = 50;
};