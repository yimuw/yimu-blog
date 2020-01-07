#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#define PRINT_NAME_VAR(var) std::cout << #var << " :" << var << std::endl;

class LucasKanadaTrackerSE2 {
public:
    // Basically, it is the se2 associated with SE2
    struct Velocity {
        float theta = 0;
        float x = 0;
        float y = 0;

        Velocity add(const Velocity& v) const
        {
            return { theta + v.theta, x + v.x, y + v.y };
        }

        Velocity neg() const
        {
            return { -theta, -x, -y };
        }

        cv::Point2f apply(const cv::Point2d& p1) const
        {
            cv::Point2f p2;
            const float sint = std::sin(theta);
            const float cost = std::cos(theta);
            p2.x = cost * p1.x - sint * p1.y + x;
            p2.y = sint * p1.x + cost * p1.y + y;
            return p2;
        }

        void print() const
        {
            std::cout << "theta:" << theta << " x:" << x << " y:" << y << std::endl;
        }
    };

    void track(const cv::Mat& cur_im_in)
    {
        cv::Mat cur_im = cur_im_in.clone();

        if (cur_features_locations_.size() == 0) {
            std::cout << "clear and detect features..." << std::endl;
            clear_and_detect_features(cur_im_in);
        }

        if (frame_count_ == 0) {
        } else {
            std::vector<cv::Point2f> new_features_locations;

            for (size_t i = 0; i < cur_features_locations_.size(); i++) {
                if (feature_within_image(cur_features_locations_.at(i))) {
                    const Velocity new_velocity = lucas_kanada_least_squares(cur_features_locations_.at(i),
                        cur_im);
                    // (x,y) = R * (0, 0) + t + loc, (0,0) because it is the center
                    new_features_locations.emplace_back(cur_features_locations_[i].x - new_velocity.x,
                        cur_features_locations_[i].y - new_velocity.y);
                } else {
                    std::cout << "ignore feature close to broader: "
                              << cur_features_locations_.at(i) << std::endl;
                }
            }

            cur_features_locations_ = new_features_locations;
        }

        last_im_ = cur_im;
        // Precompute numurical derivatives for real-time performance.
        last_im_grad_x_ = compute_derivatives(last_im_, "x");
        last_im_grad_y_ = compute_derivatives(last_im_, "y");

        ++frame_count_;
    }

    cv::Mat show_features(const std::string wname = "features",
        const size_t dt = 10)
    {
        cv::Mat im_debug = last_im_.clone();
        for (const auto& p : cur_features_locations_) {
            cv::circle(im_debug, p, 10, 240);
        }
        return im_debug;
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

private:
    void clear_and_detect_features(const cv::Mat& cur_im)
    {
        constexpr int MAX_FEATURES = 4;
        constexpr double QUALITY_LEVEL = 0.01;
        constexpr double MIN_DISTANCE = 10;

        cv::goodFeaturesToTrack(cur_im, cur_features_locations_, MAX_FEATURES, QUALITY_LEVEL, MIN_DISTANCE);
    }

    Velocity lucas_kanada_least_squares(const cv::Point2f& location,
        const cv::Mat& cur_im)
    {
        // We can also update the feature localization. Ignore for simplicity.
        Velocity optimization_vars;
        cv::Point2f feature_loc_cur_frame = location;

        constexpr size_t MAX_ITERS = 3;
        for (size_t iters = 0; iters < MAX_ITERS; ++iters) {
            const cv::Mat J = compute_jacobian(optimization_vars, feature_loc_cur_frame,
                last_im_grad_x_, last_im_grad_y_);

            const cv::Mat b = compute_b(optimization_vars, feature_loc_cur_frame,
                last_im_, cur_im);

            Velocity delta = solve_normal_equation(J, b);
            optimization_vars = optimization_vars.add(delta);
            // Minus because of dx,dy is modeled as last_x + dx = cur_x
            feature_loc_cur_frame -= { delta.x, delta.y };
            if (not feature_within_image(feature_loc_cur_frame)) {
                std::cout << "feature out of bound in GN iteration" << std::endl;
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
        const Velocity& velocity,
        const cv::Point2f& location,
        const cv::Mat& last_im_grad_x_,
        const cv::Mat& last_im_grad_y_)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat jacobian(patch_size, 3, CV_32F);

        size_t count = 0;
        // The 2 for loops can be generized by a kernal function.
        for (float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.) {
            for (float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.) {
                cv::Point2f delta(x - location.x, y - location.y);
                const auto last_p = velocity.apply(delta) + location;
                const cv::Mat jacobian_xy_wrt_se2 = compute_jacobian_of_xy_wrt_se2(delta, velocity);
                const cv::Mat jacobian_pixel_wrt_xy = (cv::Mat_<float>(1, 2)
                        << bilinear_interp(last_im_grad_x_, last_p),
                    bilinear_interp(last_im_grad_y_, last_p));
                cv::Mat jacobian_im_wrt_se2 = jacobian_pixel_wrt_xy * jacobian_xy_wrt_se2;
                assert(jacobian_im_wrt_se2.rows = 1);
                assert(jacobian_im_wrt_se2.cols = 3);
                jacobian.at<float>(count, 0) = -jacobian_im_wrt_se2.at<float>(0, 0);
                jacobian.at<float>(count, 1) = -jacobian_im_wrt_se2.at<float>(0, 1);
                jacobian.at<float>(count, 2) = -jacobian_im_wrt_se2.at<float>(0, 2);

                ++count;
            }
        }

        assert(count == patch_size);
        return jacobian;
    }

    cv::Mat compute_jacobian_of_xy_wrt_se2(
        const cv::Point2f& xy,
        const Velocity& velocity)
    {
        const float sint = std::sin(velocity.theta);
        const float cost = std::cos(velocity.theta);
        const float x = xy.x;
        const float y = xy.y;
        // Technically, it is (so2 + t2)
        // (x2, y2) = T(p) = R * (x1, y1) + t
        cv::Mat jacobian_xy_wrt_se2 = (cv::Mat_<float>(2, 3)
                << -sint * x - cost * y,
            1, 0,
            cost * x - sint * y, 0, 1.);
        return jacobian_xy_wrt_se2;
    }

    cv::Mat compute_b(
        const Velocity& velocity,
        const cv::Point2f& location,
        const cv::Mat& last_im,
        const cv::Mat& cur_im)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat b(patch_size, 1, CV_32F);

        size_t count = 0;
        for (float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.) {
            for (float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.) {
                // im(x1, y1) - im_prev(T(p)), where, (x2, y2) = T(p) = R * (x1 - l.x, y1 - l.y) + t + l
                cv::Point2f delta(x - location.x, y - location.y);
                b.at<float>(count, 0) = bilinear_interp(cur_im, { x, y })
                    - bilinear_interp(last_im, velocity.apply(delta) + location);

                ++count;
            }
        }

        assert(count == patch_size);

        return b;
    }

    Velocity solve_normal_equation(const cv::Mat& jacobian,
        const cv::Mat& b)
    {
        cv::Mat_<float> delta;

        // J^T J delta = -J^T b
        cv::solve(jacobian, -b, delta, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);
        assert(delta.rows == 3);
        assert(delta.cols == 1);

        return { delta.at<float>(0, 0), delta.at<float>(1, 0), delta.at<float>(2, 0) };
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

    uint64_t frame_count_ = 0;
    cv::Mat last_im_;
    cv::Mat last_im_grad_x_;
    cv::Mat last_im_grad_y_;
};