#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "utils.h"

struct Landmark {
    int x{ 0 };
    int y{ 0 };
};

class FaceAlignment {
public:
    void estimation_transformation(const cv::Mat& im1, const cv::Mat& im2, std::vector<Landmark>& landmarks)
    {
        landmarks_ = landmarks;

        // Want to estimate: des_im_ - warped(src_im_)
        preprocess(im1, src_im_, src_im_orig_);
        preprocess(im2, des_im_, des_im_orig_);

        rows_ = des_im_.rows;
        cols_ = des_im_.cols;

        src_im_grad_x_ = compute_derivatives(src_im_, "x");
        src_im_grad_y_ = compute_derivatives(src_im_, "y");

        warped_vis_ = cv::Mat(rows_, cols_, CV_32F, 0.);

        // grid is hardcoded for simplicity.
        // assuming the size of im is: cv::Size im_size(500, 550);
        cv::Mat identity_trans = (cv::Mat_<float>(6, 1) << 1, 0, 0, 0, 1, 0);
        cv::Mat affine_trans_global = least_squares(identity_trans, 50, cols_ - 100, 50, rows_ - 100);

        const int x_start = 100;
        const int y_start = 150;
        const int gridy_len = 350;
        const int gridx_len = 150;

        cv::Mat compare_im = des_im_orig_.clone();
        draw_landmark_in_rect(compare_im, identity_trans,
                    0, cols_,
                    0, rows_);
        cv::imwrite("compare" + std::to_string(vis_count_++) + ".png", compare_im);

        cv::Mat result = src_im_orig_.clone();
        // take middle roi
        for (int y_grid = 0; y_grid <= 0; ++y_grid) {
            for (int x_grid = 0; x_grid <= 1; ++x_grid) {

                cv::Mat affine_trans_patch = least_squares(
                    affine_trans_global,
                    x_start + x_grid * gridx_len, gridx_len,
                    y_start + y_grid * gridy_len, gridy_len);

                draw_landmark_in_rect(result, affine_trans_patch,
                    x_start + x_grid * gridx_len, gridx_len,
                    y_start + y_grid * gridy_len, gridy_len);

                cv::imshow("result", result);
                cv::imwrite("path" + std::to_string(vis_count_++) + ".png", result);
                cv::waitKey(1000);
            }
        }

        

        cv::imshow("result", result);
        cv::waitKey(0);
        cv::imwrite("path" + std::to_string(vis_count_++) + ".png", result);
    }

private:
    bool pt_in_rect(
        const cv::Point2f& p,
        const int x_start,
        const int x_len,
        const int y_start,
        const int y_len)
    {
        if (p.x >= x_start and p.x <= x_start + x_len
            and p.y >= y_start and p.y <= y_start + y_len) {
            return true;
        } else {
            return false;
        }
    }

    void draw_landmark_in_rect(
        cv::Mat& im,
        const cv::Mat& affine_trans,
        const int x_start,
        const int x_len,
        const int y_start,
        const int y_len)
    {
        for (const auto& l : landmarks_) {
            cv::Point2f p(l.x, l.y);
            if (pt_in_rect(p,
                    x_start, x_len,
                    y_start, y_len)) {
                cv::Point2f pt_in_src = apply_affine(affine_trans, p);
                // Out of bound?
                cv::circle(im, pt_in_src, 5, { 0, 255, 0 }, 3);
            }
        }
    }

    void preprocess(const cv::Mat& im, cv::Mat& result, cv::Mat& color)
    {
        cv::cvtColor(im, result, CV_BGR2GRAY);
        result.convertTo(result, CV_32F, 1 / 255.0);
        // TODO: how to hind my double chin?
        cv::Size im_size(500, 550);
        cv::resize(result, result, im_size);

        //cv::Laplacian( result, result, CV_32F, 3);

        cv::resize(im, color, im_size);

        constexpr bool APPLY_SPLINE = true;
        if (APPLY_SPLINE) {
            cv::GaussianBlur(result, result, cv::Size(9, 9), 0, 0, cv::BORDER_DEFAULT);
        }
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

    void draw_landmarks(cv::Mat& im)
    {
        for (const auto& l : landmarks_) {
            cv::circle(im, { l.x, l.y }, 5, { 0, 255, 0 }, 2);
        }
    }

    cv::Mat least_squares(
        const cv::Mat& affine_trans_init,
        const int x_start,
        const int x_len,
        const int y_start,
        const int y_len)
    {
        // row major
        cv::Mat affine_trans = affine_trans_init.clone();

        constexpr size_t MAX_ITERS = 100;
        for (size_t iters = 0; iters < MAX_ITERS; ++iters) {
            const cv::Mat J = compute_jacobian(affine_trans,
                src_im_grad_x_, src_im_grad_y_, x_start, x_len, y_start, y_len);

            const cv::Mat b = compute_b(affine_trans,
                x_start, x_len, y_start, y_len);

            cv::Mat delta = solve_normal_equation(J, b);
            affine_trans = affine_trans + delta;

            if (true) {
                if (false) {
                    std::cout << "delta trans: " << delta << std::endl;
                    std::cout << "norm:" << cv::norm(delta) << std::endl;
                    std::cout << "affine_trans: " << affine_trans << std::endl;
                }

                // TODO: why doesn't work?
                // cv::Mat transfrom = affine_trans.reshape(2, 3);
                cv::Mat transfrom = (cv::Mat_<float>(2, 3)
                        << affine_trans.at<float>(0, 0),
                    affine_trans.at<float>(1, 0),
                    affine_trans.at<float>(2, 0),
                    affine_trans.at<float>(3, 0),
                    affine_trans.at<float>(4, 0),
                    affine_trans.at<float>(5, 0));

                // cv::warpAffine(src_im_, warped, transfrom, src_im_.size(), cv::WARP_INVERSE_MAP);
                warp(src_im_, warped_vis_, affine_trans, x_start, x_len, y_start, y_len);
                draw_landmarks(warped_vis_);
                draw_landmarks(des_im_);
                cv::Mat concat;
                cv::hconcat(des_im_, warped_vis_, concat);
                cv::imshow("current result", concat);
                cv::imwrite("concat" + std::to_string(vis_count_++) + ".png", 255. * concat.clone());
                if (cv::norm(delta) < 1e-1) {
                    cv::waitKey(1000);
                } else {
                    cv::waitKey(1);
                }
            }

            if (cv::norm(delta) < 1e-1) {
                std::cout << "converged" << std::endl;
                break;
            }
        }

        std::cout << "final trans: " << affine_trans << std::endl;

        return affine_trans;
    }

    cv::Mat compute_jacobian(
        const cv::Mat& affine_trans,
        const cv::Mat& last_im_grad_x_,
        const cv::Mat& last_im_grad_y_,
        const int x_start,
        const int x_len,
        const int y_start,
        const int y_len)
    {
        assert(x_start + x_len <= cols_);
        assert(y_start + y_len <= rows_);

        cv::Mat jacobian(x_len * y_len, 6, CV_32F, 0.);

        double count = 0;
        for (float y = y_start; y < y_start + y_len; ++y) {
            for (float x = x_start; x < x_start + x_len; ++x) {
                const cv::Point2f p_trans = apply_affine(affine_trans, { x, y });
                if (inbound(p_trans)) {
                    const cv::Mat jacobian_pixel_wrt_xy = (cv::Mat_<float>(1, 2)
                            << bilinear_interp(last_im_grad_x_, p_trans),
                        bilinear_interp(last_im_grad_y_, p_trans));
                    const cv::Mat jacobian_wrt_affine = jacobian_xy_wrt_affine({ x, y });
                    const cv::Mat jacobian_im_wrt_affine = jacobian_pixel_wrt_xy * jacobian_wrt_affine;

                    assert(jacobian_im_wrt_affine.cols == 6);
                    assert(jacobian_im_wrt_affine.rows == 1);
                    for (int k = 0; k < 6; ++k) {
                        jacobian.at<float>(count, k) = -jacobian_im_wrt_affine.at<float>(0, k);
                    }
                    ++count;
                }
            }
        }

        jacobian = jacobian / sqrt(count);
        return jacobian;
    }

    //   |a b| x + e
    //   |e d| y   f
    //
    cv::Mat jacobian_xy_wrt_affine(
        const cv::Point2f& xy)
    {
        const float x = xy.x;
        const float y = xy.y;
        cv::Mat jacobian_wrt_affine = (cv::Mat_<float>(2, 6)
                << x,
            y, 1, 0, 0, 0,
            0, 0, 0, x, y, 1);
        return jacobian_wrt_affine;
    }

    cv::Mat compute_b(
        const cv::Mat& affine_trans,
        const int x_start,
        const int x_len,
        const int y_start,
        const int y_len)
    {
        assert(x_start + x_len <= cols_);
        assert(y_start + y_len <= rows_);

        // may contain empty entry. But it's fine mathematically.
        cv::Mat b(x_len * y_len, 1, CV_32F, 0.);

        double count = 0;
        for (float y = y_start; y < y_start + y_len; ++y) {
            for (float x = x_start; x < x_start + x_len; ++x) {
                cv::Point2f p_trans = apply_affine(affine_trans, { x, y });
                if (inbound(p_trans)) {
                    b.at<float>(count, 0) = bilinear_interp(des_im_, { x, y })
                        - bilinear_interp(src_im_, p_trans);

                    ++count;
                }
            }
        }

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

    cv::Mat des_im_orig_;
    cv::Mat src_im_orig_;

    // a little bit hack for visualization
    cv::Mat warped_vis_;
    int vis_count_{ 0 };
};