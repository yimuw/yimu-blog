#include <iostream>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lucas_kanada_se2_algorithm.h"

struct RotationRectSimulator {
    cv::Mat read_next()
    {
        dynamic();

        cv::Mat image = cv::Mat::zeros(image_length_, image_length_, CV_32F);

        std::vector<cv::Point> roi = compute_roi();

        // TODO: {} initailization doesn't work???
        //       cv::drawContours( image, {roi}, 0, 1);
        std::vector<std::vector<cv::Point>> co_ordinates;
        co_ordinates.push_back(std::vector<cv::Point>());
        co_ordinates[0].push_back(roi[0]);
        co_ordinates[0].push_back(roi[1]);
        co_ordinates[0].push_back(roi[2]);
        co_ordinates[0].push_back(roi[3]);
        cv::drawContours(image, co_ordinates, 0, 100, CV_FILLED);

        // more region with gredient.
        cv::GaussianBlur(image, image, cv::Size(11, 11), 0, 0, cv::BORDER_DEFAULT);

        if (false) {
            imshow("generated images", image);
            cv::waitKey(20);
        }

        ++idx;
        return image;
    }

    void dynamic()
    {
        t_x_ += 2 + xy_rand_(generator_);
        t_y_ += 1 + xy_rand_(generator_);
        t_theta_ += 0.05 + theta_rand_(generator_);
    }

    std::vector<cv::Point> compute_roi() const
    {
        cv::Point2f p1(rect_half_length_, rect_half_length_);
        cv::Point2f p2(-rect_half_length_, rect_half_length_);
        cv::Point2f p3(-rect_half_length_, -rect_half_length_);
        cv::Point2f p4(rect_half_length_, -rect_half_length_);
        const std::vector<cv::Point2f> p_origin = { p1, p2, p3, p4 };

        const auto ct = std::cos(t_theta_);
        const auto st = std::sin(t_theta_);

        std::vector<cv::Point> result;
        for (const auto& p : p_origin) {
            result.emplace_back(ct * p.x - st * p.y + t_x_ + center_.x,
                st * p.x + ct * p.y + t_y_ + center_.y);
        }
        return result;
    }

    bool has_more() const
    {
        if (idx > 1000) {
            return false;
        }

        const std::vector<cv::Point> roi = compute_roi();
        for (const auto& p : roi) {
            if (p.x < 0 || p.x >= image_length_ || p.y < 0 || p.y >= image_length_) {
                return false;
            }
        }

        return true;
    }

    size_t idx = 0;

    // rand
    std::default_random_engine generator_;
    std::normal_distribution<double> xy_rand_{ 0.0, 1. };
    std::normal_distribution<double> theta_rand_{ 0.0, 0.01 };

    // simulation a moving rect
    float t_x_ = 0;
    float t_y_ = 0;
    float t_theta_ = 0;
    // rect  const
    const float image_length_ = 800;
    const cv::Point2f center_ = { 100, 100 };
    const float rect_half_length_ = 50;
};

int main(int argc, char* argv[])
{
    RotationRectSimulator imageio;

    LucasKanadaTrackerSE2 lk_tracker;
    cv::VideoWriter video("lk-se2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 24, 
        cv::Size(imageio.image_length_, imageio.image_length_));

    while (imageio.has_more()) {
        const auto image = imageio.read_next();

        constexpr int IMAGE_DT_MS = 1;
        lk_tracker.track(image);

        const cv::Mat gray_frame = lk_tracker.show_features("LK se(2) tracker", IMAGE_DT_MS);
        
        cv::Mat frame;
        gray_frame.convertTo(frame, CV_8U);
        cv::imshow("LK se(2) tracker", frame);
        cv::waitKey(IMAGE_DT_MS);
        cv::cvtColor(frame, frame, CV_GRAY2BGR);
        video.write(frame);
    }
    video.release();

    return 0;
}
