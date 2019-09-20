#include <vector>

#include <opencv2/opencv.hpp>

class LucasKanadaTracker
{
public:
    LucasKanadaTracker()
    {
    }

    void track(const cv::Mat &cur_im)
    {
        if(frame_count_ == 0)
        {
            // constexpr int MAX_FEATURES = 1;
            // constexpr double QUALITY_LEVEL = 0.1;
            // constexpr double MIN_DISTANCE = 20;

            // cv::goodFeaturesToTrack(cur_im, cur_features_locations_, MAX_FEATURES, QUALITY_LEVEL, MIN_DISTANCE);
            // cur_features_velocities_ = std::vector<cv::Point2f>(cur_features_locations_.size(), cv::Point2f(0,0.));

            cur_features_locations_ = {{200, 200.}};
            cur_features_velocities_ = {{0, 0.}};
        }
        else
        {
            assert(cur_features_locations_.size() == cur_features_velocities_.size());

            std::vector<cv::Point2f> new_features_locations;
            std::vector<cv::Point2f> new_velocities;

            for(cv::Point2f &x_i: cur_features_locations_)
            for(size_t i = 0; i < cur_features_locations_.size(); i++)
            {
                cv::Point2f new_velocity = lucas_kanada_least_squres(cur_features_velocities_.at(i), cur_features_locations_.at(i), cur_im);

                new_velocities.push_back(new_velocity);
                new_features_locations.emplace_back(cur_features_locations_[i].x - new_velocity.x, 
                                          cur_features_locations_[i].y - new_velocity.y);

            }

            cur_features_locations_ = new_features_locations;
            cur_features_velocities_ = new_velocities;
        }

        last_im_ = cur_im;

        constexpr int KERNAL_SIZE = 3;
        Sobel( last_im_, last_im_grad_x_, CV_16S, 1, 0, KERNAL_SIZE);
        Sobel( last_im_, last_im_grad_y_, CV_16S, 0, 1, KERNAL_SIZE);
    
        ++frame_count_;
    }

    void show_features(const std::string wname = "features",
                      const size_t dt = 50)
    {
        cv::Mat im_debug = last_im_.clone();
        for(const auto &p : cur_features_locations_)
        {
            // std::cout << "feature p: " << p << std::endl;
            cv::circle(im_debug, p, 10, 240);
        }
        cv::imshow(wname, im_debug);
        cv::waitKey(dt);
    }

private:

    cv::Point2f lucas_kanada_least_squres(const cv::Point2f &velocity, const cv::Point2f &location, const cv::Mat &cur_im)
    {
        // cv::Point2f optimization_vars = velocity;
        cv::Point2f optimization_vars = {0,0.};

        for(size_t iters = 0; iters < 5; ++iters)
        {
            const cv::Mat J = compute_jacobian(optimization_vars, location, last_im_grad_x_, last_im_grad_y_);
            const cv::Mat b = compute_b(optimization_vars, location, last_im_, cur_im);

            cv::Point2f delta = solve_normal_equation(J, b);
            std::cout << "delta: " << delta << std::endl;

            optimization_vars += delta;
        }

        return optimization_vars;
    }

    cv::Mat compute_jacobian(const cv::Point2f &velocity, 
                          const cv::Point2f &location, 
                          const cv::Mat &last_im_grad_x_, 
                          const cv::Mat &last_im_grad_y_)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat jacobian(patch_size, 2, CV_32F);

        size_t count = 0;
        for(float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.)
        {
            for(float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.)
            {
                // std::cout << "x y" << x << " " << y << std::endl;
                const float last_x = x + velocity.x;
                const float last_y = y + velocity.y;

                // dx
                jacobian.at<float>(count, 0) = -bilinear_interp(last_im_grad_x_, {last_x, y});
                // dy
                jacobian.at<float>(count, 1) = -bilinear_interp(last_im_grad_y_, {x, last_y});
                ++count;
            }
        }

        std::cout << "Count: " << count << std::endl;
        
        assert(count == patch_size);
        return jacobian;
    }

    cv::Mat compute_b(const cv::Point2f &velocity, 
                          const cv::Point2f &location, 
                          const cv::Mat &last_im, 
                          const cv::Mat &cur_im)
    {
        const int patch_size = (half_window_size_ * 2 + 1) * (half_window_size_ * 2 + 1);
        cv::Mat b(patch_size, 1, CV_32F);

        size_t count = 0;
        for(float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1)
        {
            for(float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1)
            {
                const float last_x = x + velocity.x;
                const float last_y = y + velocity.y;

                
                b.at<float>(count, 0) = cur_im.at<uchar>(x,y) - bilinear_interp(last_im, {last_x, last_y});

                // std::cout << "diff1: " << b.at<float>(count, 0) << std::endl;
                // std::cout << "diff2: " << cur_im.at<uchar>(y,x) - last_im.at<uchar>(y,x) << std::endl;;
                
                // b.at<float>(count, 0) = cur_im.at<uchar>(y,x) - last_im.at<uchar>(y,x);
                ++count;
            }
        }

        assert(count == patch_size);

        return b; 
    }

    cv::Point2f solve_normal_equation(const cv::Mat &jacobian, const cv::Mat &b)
    {
        cv::Mat_<float> delta;
        // J^T J delta = J^T b
        cv::solve(jacobian, b, delta, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);

        return {delta.at<float>(0,0), delta.at<float>(1,0)};
    }

    // https://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
    inline uchar bilinear_interp(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);

        const int x = static_cast<int>(pt.x);
        const int y = static_cast<int>(pt.y);

        const int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
        const int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
        const int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
        const int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

        const float a = pt.x - static_cast<float>(x);
        const float c = pt.y - static_cast<float>(y);

        uchar val = cvRound((img.at<uchar>(y0, x0) * (1.f - a) + img.at<uchar>(y0, x1) * a) * (1.f - c)
                            + (img.at<uchar>(y1, x0) * (1.f - a) + img.at<uchar>(y1, x1) * a) * c);

        return val;
    }

    // config
    float half_window_size_ = 10;

    // states
    std::vector<cv::Point2f> cur_features_locations_;
    std::vector<cv::Point2f> cur_features_velocities_;

    uint64_t frame_count_ = 0;
    cv::Mat last_im_;
    cv::Mat last_im_grad_x_;
    cv::Mat last_im_grad_y_;
};