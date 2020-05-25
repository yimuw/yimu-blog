#pragma once

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "soft_matting.h"

class Dehaze {
public:
    struct Config {
    };

    using index_and_intensity = std::pair<cv::Point2d, float>;

    Dehaze() = default;

    void dehaze(const cv::Mat& haze_im)
    {
        const cv::Mat dark_channel = find_dark_channel(haze_im, 2);
        const cv::Vec3f atmospheric_light = compute_atmospheric_light(dark_channel, haze_im);
        const cv::Mat t = compute_t(haze_im, atmospheric_light);

        std::shared_ptr<SoftMatting> sm;
        std::string softmatting_option = "guided_filter";
        if (softmatting_option == "guided_filter") {
            sm = std::make_shared<GuideFilter>(haze_im, t);
        } else if (softmatting_option == "elimination") {
            sm = std::make_shared<SoftMattingEliminated>(haze_im, t);
        } else if (softmatting_option == "direct") {
            sm = std::make_shared<SoftMattingDirect>(haze_im, t);
        } else {
            assert(false && "unknow option");
        }

        const cv::Mat t_sm = sm->run();
        const cv::Mat dehazed = recover_J(haze_im, t_sm, atmospheric_light);

        cv::imwrite(sm->config_to_string() + ".png", 255. * dehazed);

        if (true) {
            cv::imshow("dehazed", dehazed);

            cv::Mat exposed = exposure_change(dehazed, 1.3);
            cv::imshow("exposed", exposed);

            cv::waitKey();
        }
    }

    cv::Mat recover_J(const cv::Mat& haze_im_in, const cv::Mat& t_in, const cv::Vec3f& atmospheric_light)
    {
        cv::Mat t = t_in.clone();
        cv::Mat haze_im = haze_im_in.clone();
        constexpr float MIN_T = 0.1;

        t.setTo(MIN_T, t < MIN_T);

        cv::Mat J(t.rows, t.cols, CV_32FC3);

        for (int row_idx = 0; row_idx < t.rows; ++row_idx) {
            for (int col_idx = 0; col_idx < t.cols; ++col_idx) {
                cv::Vec3f haze_bgr = haze_im.at<cv::Vec3f>(row_idx, col_idx);
                float robust_t = std::max(t.at<float>(row_idx, col_idx), MIN_T);
                J.at<cv::Vec3f>(row_idx, col_idx) = (haze_bgr - atmospheric_light) / robust_t + atmospheric_light;
            }
        }

        return J;
    }

    cv::Mat compute_t(const cv::Mat& haze_im, const cv::Vec3f& atmospheric_light)
    {
        cv::Mat bgr[3];
        cv::split(haze_im, bgr);
        bgr[0] = bgr[0] / atmospheric_light[0];
        bgr[1] = bgr[1] / atmospheric_light[1];
        bgr[2] = bgr[2] / atmospheric_light[2];

        cv::Mat normalized;
        cv::merge(bgr, 3, normalized);

        constexpr float OMAGE = 0.80;
        const cv::Mat t = 1 - OMAGE * find_dark_channel(normalized, 3);

        if (false) {
            cv::imshow("t map", t);
            cv::waitKey();
        }

        return t;
    }

    cv::Mat find_dark_channel(const cv::Mat& m, const int window_half_size)
    {

        cv::Mat m_pad;
        cv::copyMakeBorder(m, m_pad, window_half_size, window_half_size,
            window_half_size, window_half_size, cv::BORDER_REPLICATE);

        if (false) {
            cv::imshow("m_pad", m_pad);
            cv::waitKey(0);
        }

        cv::Mat dark_chaneel = cv::Mat(m.rows, m.cols, CV_32FC1, 0.);

        for (int row_idx = window_half_size; row_idx < m_pad.rows - window_half_size; ++row_idx) {
            for (int col_idx = window_half_size; col_idx < m_pad.cols - window_half_size; ++col_idx) {
                dark_chaneel.at<float>(row_idx - window_half_size, col_idx - window_half_size)
                    = find_min_in_rbg_patch(m_pad, row_idx, col_idx, window_half_size);
            }
        }

        if (false) {
            cv::imshow("dark_chaneel", dark_chaneel);
            // cv::waitKey(0);
        }

        return dark_chaneel;
    }

    inline float find_min_in_rbg_patch(const cv::Mat& m, const int row, const int col, const int window_half_size)
    {
        float min_val = std::numeric_limits<float>::max();
        for (int dy = -window_half_size; dy <= window_half_size; ++dy) {
            for (int dx = -window_half_size; dx <= window_half_size; ++dx) {
                const cv::Vec3f gbr = m.at<cv::Vec3f>(row + dy, col + dx);
                min_val = std::min(gbr[0], min_val);
                min_val = std::min(gbr[1], min_val);
                min_val = std::min(gbr[2], min_val);
            }
        }
        return min_val;
    }

    cv::Vec3f compute_atmospheric_light(const cv::Mat& dark_channel, const cv::Mat& haze_im)
    {
        std::vector<index_and_intensity> dark_channel_index_and_intensities;

        for (int row_idx = 0; row_idx < dark_channel.rows; ++row_idx) {
            for (int col_idx = 0; col_idx < dark_channel.cols; ++col_idx) {
                float intensity = dark_channel.at<float>(row_idx, col_idx);
                dark_channel_index_and_intensities.emplace_back(
                    cv::Point2d(col_idx, row_idx), intensity);
            }
        }

        std::sort(dark_channel_index_and_intensities.begin(), dark_channel_index_and_intensities.end(),
            [](const index_and_intensity& l, const index_and_intensity& r) {
                return l.second < r.second;
            });

        std::vector<index_and_intensity> candidate_from_dark(
            dark_channel_index_and_intensities.end() - dark_channel_index_and_intensities.size() * 0.0005,
            dark_channel_index_and_intensities.end());

        // show_pixel(haze_im, candidate_from_dark, "candidate_from_dark");

        // Not the same as in the paper
        cv::Vec3f mean(0., 0., 0.);
        for (const auto& p : candidate_from_dark) {
            mean += haze_im.at<cv::Vec3f>(p.first) / (float)candidate_from_dark.size();
        }
        return mean;
    }

    void show_pixel(const cv::Mat& im,
        const std::vector<index_and_intensity>& candidate_from_dark,
        const std::string& info)
    {
        cv::Mat im_viz = im.clone();
        for (const auto& p : candidate_from_dark) {
            im_viz.at<cv::Vec3f>(p.first) = cv::Vec3f(0, 0, 1.);
        }

        cv::imshow(info, im_viz);
        cv::waitKey();
    }

    cv::Mat exposure_change(const cv::Mat& image, const float alpha, const float beta = 0)
    {
        assert(image.type() == CV_32FC3);
        cv::Mat new_image(image.rows, image.cols, CV_32FC3, 0.);
        for (int y = 0; y < image.rows; y++) {
            for (int x = 0; x < image.cols; x++) {
                for (int c = 0; c < image.channels(); c++) {
                    new_image.at<cv::Vec3f>(y, x)[c]
                        = std::min(0.999f, alpha * image.at<cv::Vec3f>(y, x)[c] + beta);
                }
            }
        }
        return new_image;
    }
};