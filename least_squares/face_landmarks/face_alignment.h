#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#define PRINT_NAME_VAR(var) std::cout << #var << " :" << var << std::endl;

struct Landmark {
    int x {0};
    int y {0};
};

class FaceAlignment {
public:
    void estimation_transformation(const cv::Mat& im1, const cv::Mat& im2, std::vector<Landmark>& landmarks)
    {
        landmarks_ = landmarks;

        // Want to estimate: des_im_ - warped(last_im_)
        src_im_ = preprocess(im1);
        des_im_ = preprocess(im2);

        // cv::Mat temp;
        // cv::Size im_size(500, 550);
        // cv::resize(im2, temp, im_size);
        // imwrite( "des_im.jpg", temp );
    
        rows_ = des_im_.rows;
        cols_ = des_im_.cols;

        src_im_grad_x_ = compute_derivatives(src_im_, "x");
        src_im_grad_y_ = compute_derivatives(src_im_, "y");

        cv::Mat homo_trans = least_squares();
    }

private:
    cv::Mat preprocess(const cv::Mat& im)
    {
        cv::Mat result;

        cv::cvtColor(im, result, CV_BGR2GRAY);
        result.convertTo(result, CV_32F, 1 / 255.0);
        // TODO: how to hind my double chin?
        cv::Size im_size(500, 550);
        cv::resize(result, result, im_size);

        //cv::Laplacian( result, result, CV_32F, 3);

        constexpr bool APPLY_SPLINE = true;
        if (APPLY_SPLINE) {
            cv::GaussianBlur(result, result, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);
        }
        return result;
    }

    cv::Mat compute_derivatives(const cv::Mat& src, const std::string& type)
    {
        cv::Mat result;

        if (type == "x") {
            // Note: filter2d flip the image patch
            cv::Mat x_grad_kernal = (cv::Mat_<double>(3, 3) << 0, 0, 0, // NOLINT
                -0.5, 0, 0.5, // NOLINT
                0, 0, 0); // NOLINT
            cv::filter2D(src, result, CV_32F, x_grad_kernal);

            assert(std::abs(result.at<float>(10, 10) - (src.at<float>(10, 11) - src.at<float>(10, 9)) / 2.)
                    < 1e-8
                && "x gradient checking failed");
        } else if (type == "y") {
            // Note: filter2d flip the image patch
            cv::Mat y_grad_kernal = (cv::Mat_<double>(3, 3) << 0, -0.5, 0, // NOLINT
                0, 0, 0, // NOLINT
                0, 0.5, 0); // NOLINT
            cv::filter2D(src, result, CV_32F, y_grad_kernal);

            assert(std::abs(result.at<float>(10, 10) - (src.at<float>(11, 10) - src.at<float>(9, 10)) / 2.)
                    < 1e-8
                && "y gradient checking failed");
        } else {
            assert(false && "invalid input");
        }

        return result;
    }

    cv::Point2f apply_homo(const cv::Mat& trans, const cv::Point2f& p)
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

    void draw_landmarks(cv::Mat &im) {
        for(const auto &l : landmarks_) {
            cv::circle(im, {l.x, l.y}, 5, {0,255,0}, 1);
        }
    }

    cv::Mat least_squares()
    {
        // row major
        cv::Mat homo_trans = (cv::Mat_<float>(6, 1) << 1, 0, 0, 0, 1, 0);

        constexpr size_t MAX_ITERS = 100;
        for (size_t iters = 0; iters < MAX_ITERS; ++iters) {
            const cv::Mat J = compute_jacobian(homo_trans,
                src_im_grad_x_, src_im_grad_y_);

            const cv::Mat b = compute_b(homo_trans,
                src_im_, des_im_);

            cv::Mat delta = solve_normal_equation(J, b);
            homo_trans = homo_trans + delta;

            if (cv::norm(delta) < 1e-6) {
                std::cout << "converged" << std::endl;
                break;
            }

            if (true) {
                std::cout << "delta trans: " << delta << std::endl;
                std::cout << "norm:" << cv::norm(delta) << std::endl;
                std::cout << "homo_trans: " << homo_trans << std::endl;

                // TODO: why doesn't work?
                // cv::Mat transfrom = homo_trans.reshape(2, 3);
                cv::Mat transfrom = (cv::Mat_<float>(2, 3)
                        << homo_trans.at<float>(0, 0),
                    homo_trans.at<float>(1, 0),
                    homo_trans.at<float>(2, 0),
                    homo_trans.at<float>(3, 0),
                    homo_trans.at<float>(4, 0),
                    homo_trans.at<float>(5, 0));

                cv::Mat warped;
                cv::warpAffine(src_im_, warped, transfrom, src_im_.size(), cv::WARP_INVERSE_MAP);
                draw_landmarks(warped);
                draw_landmarks(des_im_);
                cv::Mat concat;
                cv::hconcat(des_im_, warped, concat);
                cv::imshow("current result", concat);
                cv::waitKey(1);
            }
        }

        std::cout << "final trans: " << homo_trans << std::endl;

        return homo_trans;
    }

    inline bool point_in_roi(const cv::Point2f &p) {
        const cv::Point2f center(src_im_.cols / 2, src_im_.rows / 2);
        const float roi_r = 0.95 * std::min(center.x, center.y);

        if(cv::norm(center - p) < roi_r) {
            return true;
        } else {
            return false;
        }
    }

    cv::Mat compute_jacobian(
        const cv::Mat& homo_trans,
        const cv::Mat& last_im_grad_x_,
        const cv::Mat& last_im_grad_y_)
    {
        cv::Mat jacobian(rows_ * cols_, 6, CV_32F, 0.);

        double count = 0;
        for (float y = 0; y < rows_; ++y) {
            for (float x = 0; x < cols_; ++x) {
                const cv::Point2f p_trans = apply_homo(homo_trans, { x, y });
                if (inbound(p_trans) and point_in_roi(p_trans)) {
                    const cv::Mat jacobian_pixel_wrt_xy = (cv::Mat_<float>(1, 2)
                            << bilinear_interp(last_im_grad_x_, p_trans),
                        bilinear_interp(last_im_grad_y_, p_trans));
                    const cv::Mat jacobian_wrt_homo = jacobian_xy_wrt_homo({ x, y });
                    const cv::Mat jacobian_im_wrt_homo = jacobian_pixel_wrt_xy * jacobian_wrt_homo;

                    // PRINT_NAME_VAR(jacobian_pixel_wrt_xy);
                    // PRINT_NAME_VAR(jacobian_wrt_homo);

                    assert(jacobian_im_wrt_homo.cols == 6);
                    assert(jacobian_im_wrt_homo.rows == 1);
                    for (int k = 0; k < 6; ++k) {
                        jacobian.at<float>(count, k) = -jacobian_im_wrt_homo.at<float>(0, k);
                    }
                    ++count;
                }
            }
        }
        // std::cout << "j: " << jacobian << std::endl;
        // std::cout << "count: " << count << std::endl;

        jacobian = jacobian / sqrt(count);
        return jacobian;
    }

    //   |a b| x + e
    //   |e d| y   f
    //
    cv::Mat jacobian_xy_wrt_homo(
        const cv::Point2f& xy)
    {
        const float x = xy.x;
        const float y = xy.y;
        cv::Mat jacobian_wrt_homo = (cv::Mat_<float>(2, 6)
                << x,
            y, 1, 0, 0, 0,
            0, 0, 0, x, y, 1);
        return jacobian_wrt_homo;
    }

    cv::Mat compute_b(
        const cv::Mat& homo_trans,
        const cv::Mat& last_im,
        const cv::Mat& cur_im)
    {
        // may contain empty entry. But it's fine mathematically.
        cv::Mat b(rows_ * cols_, 1, CV_32F, 0.);

        double count = 0;
        for (float y = 0; y < rows_; ++y) {
            for (float x = 0; x < cols_; ++x) {
                cv::Point2f p_trans = apply_homo(homo_trans, { x, y });
                if (inbound(p_trans) and point_in_roi(p_trans)) {
                    b.at<float>(count, 0) = bilinear_interp(cur_im, { x, y })
                        - bilinear_interp(last_im, p_trans);

                    ++count;
                }
            }
        }
        // std::cout << "b: " << b << std::endl;
        // std::cout << "count:" << count << std::endl;
        b = b / sqrt(count);
        return b;
    }

    inline bool inbound(const cv::Point2f& p)
    {
        const float x = p.x;
        const float y = p.y;
        if (y >= 0 and y < rows_ and x >= 0 and x < cols_) {
            return true;
        } else {
            return false;
        }
    }

    cv::Mat solve_normal_equation(const cv::Mat& jacobian,
        const cv::Mat& b)
    {
        cv::Mat_<float> delta;

        // J^T J delta = -J^T b
        cv::solve(jacobian, -b, delta, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);
        assert(delta.rows == 6);
        assert(delta.cols == 1);

        return delta;
    }

    // https://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
    inline float bilinear_interp(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);
        // ceil
        const int x = static_cast<int>(pt.x);
        const int y = static_cast<int>(pt.y);

        const int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_DEFAULT);
        const int x1 = cv::borderInterpolate(x + 1, img.cols, cv::BORDER_DEFAULT);
        const int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_DEFAULT);
        const int y1 = cv::borderInterpolate(y + 1, img.rows, cv::BORDER_DEFAULT);

        const float dx = pt.x - static_cast<float>(x);
        const float dy = pt.y - static_cast<float>(y);

        assert(dx >= 0 && dy >= 0 && "dx dy less than 0");

        const float v1 = img.at<float>(y0, x0);
        const float v2 = img.at<float>(y0, x1);
        const float v3 = img.at<float>(y1, x0);
        const float v4 = img.at<float>(y1, x1);

        const float val = (v1 * (1. - dx) + v2 * dx) * (1 - dy)
            + (v3 * (1. - dx) + v4 * dx) * dy;

        return val;
    }

    int rows_{ 0 };
    int cols_{ 0 };

    std::vector<Landmark> landmarks_;

    cv::Mat des_im_;
    cv::Mat src_im_;
    cv::Mat src_im_grad_x_;
    cv::Mat src_im_grad_y_;
};