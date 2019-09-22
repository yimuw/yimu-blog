#include <vector>

#include <opencv2/opencv.hpp>

class LucasKanadaTracker
{
public:
    LucasKanadaTracker()
    {
    }

    void track(const cv::Mat &cur_im_in)
    {
        cv::Mat cur_im;
        cv::GaussianBlur( cur_im_in, cur_im, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

        if(frame_count_ == 0)
        {
            // constexpr int MAX_FEATURES = 1;
            // constexpr double QUALITY_LEVEL = 0.1;
            // constexpr double MIN_DISTANCE = 20;

            // cv::goodFeaturesToTrack(cur_im, cur_features_locations_, MAX_FEATURES, QUALITY_LEVEL, MIN_DISTANCE);
            // cur_features_velocities_ = std::vector<cv::Point2f>(cur_features_locations_.size(), cv::Point2f(0,0.));

            cur_features_locations_ = {{160, 130.}};
            cur_features_velocities_ = {{0, 0.}};
        }
        else
        {
            assert(cur_features_locations_.size() == cur_features_velocities_.size());

            std::vector<cv::Point2f> new_features_locations;
            std::vector<cv::Point2f> new_velocities;

            for(size_t i = 0; i < cur_features_locations_.size(); i++)
            {
                cv::Point2f new_velocity = lucas_kanada_least_sqaures(cur_features_velocities_.at(i), 
                                                                     cur_features_locations_.at(i), 
                                                                     cur_im);

                new_velocities.push_back(new_velocity);
                new_features_locations.emplace_back(cur_features_locations_[i].x - new_velocity.x, 
                                          cur_features_locations_[i].y - new_velocity.y);

            }

            cur_features_locations_ = new_features_locations;
            cur_features_velocities_ = new_velocities;
        }

        last_im_ = cur_im;

        // // precompute grad for last image
        // constexpr int KERNAL_SIZE = 3;
        // cv::Sobel( last_im_, last_im_grad_x_, CV_16S, 1, 0, KERNAL_SIZE);
        // cv::Sobel( last_im_, last_im_grad_y_, CV_16S, 0, 1, KERNAL_SIZE);

        last_im_grad_x_ = compute_derivatives(last_im_, "x");
        last_im_grad_y_ = compute_derivatives(last_im_, "y");

    
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

    cv::Mat compute_derivatives(const cv::Mat &src, const std::string &type)
    {
        cv::Mat result;

        if(type == "x")
        {
            // Note: filter2d flip the image patch
            cv::Mat x_grad_kernal = (cv::Mat_<double>(3,3) << 0, 0, 0, 
                                                              -0.5, 0, 0.5, 
                                                              0, 0, 0);
            cv::filter2D(src, result, CV_32F , x_grad_kernal);

            assert(std::abs(result.at<float>(10, 10) - (src.at<float>(10, 11) - src.at<float>(10, 9)) / 2.)
                < 1e-8 && "xgradient checking failed");
        }
        else if(type == "y")
        {
            // Note: filter2d flip the image patch
            cv::Mat y_grad_kernal = (cv::Mat_<double>(3,3) << 0, -0.5, 0, 
                                                              0, 0, 0, 
                                                              0, 0.5, 0);
            cv::filter2D(src, result, CV_32F , y_grad_kernal);

            assert(std::abs(result.at<float>(10, 10) - (src.at<float>(11, 10) - src.at<float>(9, 10)) / 2.)
                < 1e-8 && "ygradient checking failed");
        }
        else
        {
            assert(false && "invalid input");
        }

        assert(result.cols == src.cols);
        assert(result.rows == src.rows);

        if(false)
        {
            show_gradiant(result, type);
        }

        return result;

    }

    void show_gradiant(const cv::Mat &grad_im, std::string wname = "")
    {
        cv::Mat abs_grad;
        cv::convertScaleAbs( grad_im, abs_grad );
        cv::imshow( wname, abs_grad );
        cv::waitKey(0);
    }

private:

    cv::Point2f lucas_kanada_least_sqaures(const cv::Point2f &velocity, const cv::Point2f &location, const cv::Mat &cur_im)
    {
        cv::Point2f optimization_vars = velocity;
        // cv::Point2f optimization_vars = {0,0.};

        for(size_t iters = 0; iters < 3; ++iters)
        {
            std::cout << "optimization_vars: " << optimization_vars << std::endl;
            const cv::Mat J = compute_jacobian(optimization_vars, location, last_im_grad_x_, last_im_grad_y_);
            const cv::Mat b = compute_b(optimization_vars, location, last_im_, cur_im);

            cv::Point2f delta = solve_normal_equation(J, b);
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

                // std::cout << "velocity: " << velocity << std::endl;
                // std::cout << "xy out: "<<  cv::Point2f(last_x, last_y) << std::endl;

                if(true)
                {
                    // dx
                    jacobian.at<float>(count, 0) = -bilinear_interp(last_im_grad_x_, {last_x, last_y});
                    // dy
                    jacobian.at<float>(count, 1) = -bilinear_interp(last_im_grad_y_, {last_x, last_y});
                    ++count;
                }
                else
                {
                    int x = static_cast<int>(last_x);
                    int y = static_cast<int>(last_y);

                    // dx
                    jacobian.at<float>(count, 0) = -(last_im_.at<float>(y, x + 1) - last_im_.at<float>(y, x - 1)) / 2.;
                    // dy
                    jacobian.at<float>(count, 1) = -(last_im_.at<float>(y + 1, x) - last_im_.at<float>(y- 1, x)) / 2.;

                    ++count;
                }
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
        for(float y = location.y - half_window_size_; y <= location.y + half_window_size_ + 1e-6; y += 1.)
        {
            for(float x = location.x - half_window_size_; x <= location.x + half_window_size_ + 1e-6; x += 1.)
            {
                const float last_x = x + velocity.x;
                const float last_y = y + velocity.y;

                b.at<float>(count, 0) = cur_im.at<float>(y,x) - bilinear_interp(last_im, {last_x, last_y});
                
                ++count;
            }
        }

        assert(count == patch_size);

        return b; 
    }

    cv::Point2f solve_normal_equation(const cv::Mat &jacobian, const cv::Mat &b)
    {
        cv::Mat_<float> delta;
        
        // J^T J delta = -J^T b
        //cv::solve(jacobian, -b, delta, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);

        cv::Mat right = jacobian.t() * jacobian;
        cv::Mat left = - jacobian.t() * b;
        cv::solve(right, left, delta);

        std::cout << "Hessian: " << right << std::endl;
        std::cout << "Delta : " << delta << std::endl;

        return {delta.at<float>(0,0), delta.at<float>(1,0)};
    }

    // https://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
    inline float bilinear_interp(const cv::Mat& img, cv::Point2f pt)
    {
        assert(!img.empty());
        assert(img.channels() == 1);

        // ceil
        const int x = static_cast<int>(pt.x);
        const int y = static_cast<int>(pt.y);

        const int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_DEFAULT);
        const int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_DEFAULT);
        const int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_DEFAULT);
        const int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_DEFAULT);

        const float dx = pt.x - static_cast<float>(x);
        const float dy = pt.y - static_cast<float>(y);

        assert(dx >= 0 && dy >= 0 && "dx dy less then 0");

        const float v1 = img.at<float>(y0, x0);
        const float v2 = img.at<float>(y0, x1);
        const float v3 = img.at<float>(y1, x0);
        const float v4 = img.at<float>(y1, x1);
 
        const float val = (v1 * (1. - dx) + v2 * dx) * (1 - dy)
                  + (v3 * (1. - dx) + v4 * dx) * dy;

        // std::cout << "pt xy:" << pt << std::endl;
        // std::cout << "x y: " << x << ", " << y << std::endl;
        // std::cout << "x0 y0: " << x0 << ", " << y0 << std::endl;
        // std::cout << "x1 y1: " << x1 << ", " << y1 << std::endl;
        // std::cout << "v1: " << v1 << std::endl;
        // std::cout << "v2: " << v2 << std::endl;
        // std::cout << "v3: " << v3 << std::endl;
        // std::cout << "v4: " << v4 << std::endl;
        // std::cout << "val: " << val << std::endl;

        return val;
    }

    // Config
    float half_window_size_ = 20;

    // Internal states
    std::vector<cv::Point2f> cur_features_locations_;
    std::vector<cv::Point2f> cur_features_velocities_;
    uint64_t frame_count_ = 0;
    cv::Mat last_im_;
    cv::Mat last_im_grad_x_;
    cv::Mat last_im_grad_y_;
};