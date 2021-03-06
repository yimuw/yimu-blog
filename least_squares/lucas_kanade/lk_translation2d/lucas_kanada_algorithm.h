#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

class LucasKanadaTracker {
    using Hessian = cv::Mat;

public:
    LucasKanadaTracker()
    {
    }

    void track(const cv::Mat& cur_im_in)
    {
        cv::Mat im_gray;
        // To gray
        cv::cvtColor(cur_im_in, im_gray, CV_BGR2GRAY);
        // To float. Not necessary
        im_gray.convertTo(im_gray, CV_32F, 1 / 255.0);

        cv::Mat cur_im = im_gray;
        // Same as apply spline in the optimization problem.
        // TODO: prove it
        constexpr bool APPLY_SPLINE = false;
        if (APPLY_SPLINE) {
            cv::GaussianBlur(cur_im, cur_im, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
        }

        if (cur_features_locations_.size() < 50) {
            std::cout << "clear and detect features..." << std::endl;
            clear_and_detect_features(cur_im);
        }

        if (frame_count_ == 0) {
        } else {
            assert(cur_features_locations_.size() == cur_features_velocities_.size());

            std::vector<cv::Point2f> new_features_locations;
            std::vector<cv::Point2f> new_velocities;
            std::vector<Hessian> new_hessians;

            for (size_t i = 0; i < cur_features_locations_.size(); i++) {
                if (feature_within_image(cur_features_locations_.at(i)) // cur image inbound
                    // last image inbound
                    and feature_within_image(cur_features_locations_.at(i) + cur_features_velocities_.at(i))) {
                    Hessian hessian;
                    cv::Point2f new_velocity = lucas_kanada_least_squares(cur_features_velocities_.at(i),
                        cur_features_locations_.at(i),
                        cur_im,
                        &hessian);
                    // We can filter by Hessain here.
                    new_velocities.push_back(new_velocity);
                    new_features_locations.emplace_back(cur_features_locations_[i].x - new_velocity.x,
                        cur_features_locations_[i].y - new_velocity.y);
                    new_hessians.push_back(hessian);
                } else {
                    std::cout << "ignore features close to broader: "
                              << cur_features_locations_.at(i) << std::endl;
                }
            }

            cur_features_locations_ = new_features_locations;
            cur_features_velocities_ = new_velocities;
            hessians_ = new_hessians;
        }

        last_im_ = cur_im;
        last_color_im_ = cur_im_in;
        // Precompute numurical derivatives for real-time performance.
        last_im_grad_x_ = compute_derivatives(last_im_, "x");
        last_im_grad_y_ = compute_derivatives(last_im_, "y");

        ++frame_count_;
    }

    cv::Mat show_features(const std::string wname = "features",
        const size_t dt = 1)
    {
        cv::Mat im_debug = last_color_im_.clone();
        for (const auto& p : cur_features_locations_) {
            cv::circle(im_debug, p, 10, { 0, 255, 0 }, 1);
        }

        cv::imshow(wname, im_debug);
        cv::waitKey(dt);

        return im_debug;
    }

    cv::Mat show_features_with_covariance(const std::string wname = "features",
        const size_t dt = 1)
    {
        cv::Mat im_debug = last_color_im_.clone();
        assert(cur_features_locations_.size() == hessians_.size());
        for (size_t i = 0; i < hessians_.size(); ++i) {
            auto& p = cur_features_locations_[i];
            auto& hessian = hessians_[i];

            cv::Mat eigenvalues;
            cv::Mat eigenvectors;
            cv::eigen(hessian.inv(), eigenvalues, eigenvectors);

            const float theta = std::atan2(eigenvectors.at<float>(0, 1), eigenvectors.at<float>(0, 0)) / M_PI * 180.;
            constexpr float SCALE = 10;
            const int l1 = static_cast<int>(SCALE * std::sqrt(eigenvalues.at<float>(0, 0)));
            const int l2 = static_cast<int>(SCALE * std::sqrt(eigenvalues.at<float>(1, 0)));

            constexpr double MAX_STD = 50;
            if (l1 > MAX_STD) {
                // Bad tracks
                cv::ellipse(im_debug, p, { static_cast<int>(MAX_STD), static_cast<int>(l2 * MAX_STD / l1) },
                    theta, 0, 360, { 100, 0, 0 }, 1);
            } else {
                // TODO: opencv bug if thinkness is not 1?
                cv::ellipse(im_debug, p, { l1, l2 }, theta, 0, 360, { 0, 255, 0 }, 1);
            }
        }
        cv::imshow(wname, im_debug);
        cv::waitKey(dt);

        return im_debug;
    }

    void show_gradient(const cv::Mat& grad_im, std::string wname = "")
    {
        cv::Mat abs_grad;
        cv::convertScaleAbs(grad_im, abs_grad);
        cv::imshow(wname, abs_grad);
        cv::waitKey(0);
    }

private:
    void clear_and_detect_features(const cv::Mat& cur_im)
    {
        constexpr int MAX_FEATURES = 70;
        constexpr double QUALITY_LEVEL = 0.01;
        constexpr double MIN_DISTANCE = 20;

        cv::goodFeaturesToTrack(cur_im, cur_features_locations_, MAX_FEATURES, QUALITY_LEVEL, MIN_DISTANCE);
        cur_features_velocities_ = std::vector<cv::Point2f>(cur_features_locations_.size(), cv::Point2f(0, 0.));

        cv::Mat hessian_default = (cv::Mat_<double>(2, 2) << 1, 0, 0, 1.);
        hessians_ = std::vector<Hessian>(cur_features_locations_.size(), hessian_default);

        constexpr bool MANUAL_DETECTION = false;
        if (MANUAL_DETECTION) {
            cur_features_locations_ = { { 160, 130. } };
            cur_features_velocities_ = { { 0, 0. } };
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

        assert(result.cols == src.cols);
        assert(result.rows == src.rows);

        if (false) {
            show_gradient(result, type);
        }

        return result;
    }

    cv::Point2f lucas_kanada_least_squares(const cv::Point2f& velocity,
        const cv::Point2f& location,
        const cv::Mat& cur_im,
        Hessian* const hessian_ptr)
    {
        // We can also update the feature localization. Ignore for simplicity.
        cv::Point2f optimization_vars = velocity;
        cv::Point2f feature_loc_cur_frame = location;

        constexpr size_t MAX_ITERS = 3;
        for (size_t iters = 0; iters < MAX_ITERS; ++iters) {
            const cv::Mat J = compute_jacobian(optimization_vars, feature_loc_cur_frame,
                last_im_grad_x_, last_im_grad_y_);

            const cv::Mat b = compute_b(optimization_vars, feature_loc_cur_frame,
                last_im_, cur_im);

            cv::Point2f delta = solve_normal_equation(J, b, hessian_ptr);

            optimization_vars += delta;
            // Minus because of dx,dy is modeled as last_x + dx = cur_x
            feature_loc_cur_frame -= delta;

            if (not feature_within_image(feature_loc_cur_frame)) {
                std::cout << "feature out of bound in GN iteration" << std::endl;
                break;
            }

            // stop condition
            if (delta.dot(delta) < 1e-4) {
                break;
            }
        }

        return optimization_vars;
    }

    bool feature_within_image(const cv::Point2f& feature_location)
    {
        const auto& x = feature_location.x;
        const auto& y = feature_location.y;

        if (x + half_window_size_ < last_im_.cols && x - half_window_size_ >= 0
            && y + half_window_size_ < last_im_.cols && y - half_window_size_ >= 0) {
            return true;
        } else {
            return false;
        }
    }

    cv::Mat compute_jacobian(
        const cv::Point2f& velocity,
        const cv::Point2f& location,
        const cv::Mat& last_im_grad_x_,
        const cv::Mat& last_im_grad_y_)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat jacobian(patch_size, 2, CV_32F);

        size_t count = 0;
        // The 2 for loops can be generalized by a kernal function.
        for (float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.) {
            for (float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.) {
                const float last_x = x + velocity.x;
                const float last_y = y + velocity.y;
                // dx
                jacobian.at<float>(count, 0) = -bilinear_interp(last_im_grad_x_, { last_x, last_y });
                // dy
                jacobian.at<float>(count, 1) = -bilinear_interp(last_im_grad_y_, { last_x, last_y });
                ++count;
            }
        }

        assert(static_cast<int>(count) == patch_size);
        return jacobian;
    }

    cv::Mat compute_b(
        const cv::Point2f& velocity,
        const cv::Point2f& location,
        const cv::Mat& last_im,
        const cv::Mat& cur_im)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat b(patch_size, 1, CV_32F);

        size_t count = 0;
        for (float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.) {
            for (float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.) {
                const float last_x = x + velocity.x;
                const float last_y = y + velocity.y;

                b.at<float>(count, 0) = bilinear_interp(cur_im, { x, y })
                    - bilinear_interp(last_im, { last_x, last_y });

                ++count;
            }
        }

        assert(static_cast<int>(count) == patch_size);

        return b;
    }

    cv::Point2f solve_normal_equation(const cv::Mat& jacobian,
        const cv::Mat& b,
        Hessian* const hessian_ptr)
    {
        cv::Mat_<float> delta;

        // J^T J delta = -J^T b
        cv::solve(jacobian, -b, delta, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);

        // std::optional
        if (hessian_ptr != nullptr) {
            *hessian_ptr = jacobian.t() * jacobian;
        }

        return { delta.at<float>(0, 0), delta.at<float>(1, 0) };
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

    // Config
    float half_window_size_ = 14;

    // Internal states
    std::vector<cv::Point2f> cur_features_locations_;
    std::vector<cv::Point2f> cur_features_velocities_;
    std::vector<Hessian> hessians_;

    uint64_t frame_count_ = 0;
    cv::Mat last_im_;
    cv::Mat last_color_im_;
    cv::Mat last_im_grad_x_;
    cv::Mat last_im_grad_y_;
};