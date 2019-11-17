#pragma once

#include <vector>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ct_types.h"
#include "sparse_solver.h"


struct ScanLine
{
    std::vector<int> scaned_indexes;
    float accumulatedValue = 0.;
};

class CtReconstruction
{
public:
    
    cv::Mat generate_small_im()
    {
        constexpr int im_size = 50;
        im_rows_ = im_size;
        im_cols_ = im_size;
        num_variables_ = im_size * im_size;

        cv::Mat image(im_size,im_size, CV_32F, cv::Scalar(0.));

        {
            cv::Rect region_of_interest = cv::Rect(im_size / 2, im_size / 2, im_size / 4, im_size / 4);
            cv::Mat image_roi = image(region_of_interest);
            image_roi.setTo(1.);
        }
        {
            cv::Rect region_of_interest = cv::Rect(im_size / 2, im_size / 4, im_size / 4, im_size / 4);
            cv::Mat image_roi = image(region_of_interest);
            image_roi.setTo(0.5);
        }

        return image;
    }

    static inline size_t index_map(const cv::Mat &image, const double x, const double y)
    {
        return y * image.cols + x;
    }

    ScanLine ct_scan_single_line(const cv::Point2f &center, 
                                 const double theta,
                                 const cv::Mat &image)
    {
        const size_t image_size_x = image.cols;
        const size_t image_size_y = image.rows;

        assert(image_size_x * image_size_y > 0);
        assert(center.x >=0. && center.x < image_size_x && center.y >= 0. && center.y < image_size_y);

        // convert (center, theta) to y = a * x + b
        const double a = std::tan(theta);
        const double b = center.y - center.x * a;

        ScanLine scan_line;

        // Search for lambda function if you don't understand
        auto scan_pixel = [&scan_line, &image](const double x, const double y)
        {
            if(x < image.cols and y < image.rows and x >= 0 and y >= 0)
            {
                scan_line.scaned_indexes.push_back(index_map(image, x, y));
                scan_line.accumulatedValue += image.at<float>(y, x);
            }
        };

        // It is a classical computer graphic problem.
        // I try to solve the problem when a = 999, loop through x only pass 1 pixel.
        // Loop x
        constexpr double HALF_SPACE_A = 0.5;

        if(std::abs(a) < HALF_SPACE_A)
        {
            for(size_t x = 0; x < image_size_x; ++x)
            {
                const int y = static_cast<int>(a * x + b);
                scan_pixel(x,y);
            }
        }
        // Loop y
        else
        {
            for(size_t y = 0; y < image_size_y; ++y)
            {
                const int x = static_cast<int>((y - b) / a);
                scan_pixel(x,y);
            }
        }

        assert(scan_line.scaned_indexes.size() > 0);

        return scan_line;
    }

    std::vector<ScanLine> ct_scan_random(const cv::Mat &image)
    {
        const size_t image_size_x = image.cols;
        const size_t image_size_y = image.rows; 

        std::vector<ScanLine> scan_lines;

        for(int x = 0; x < im_cols_; ++x)
        {
            scan_lines.emplace_back(ct_scan_single_line({static_cast<float>(x), 0.}, M_PI / 2, image));
        }
        for(int y = 0; y < im_rows_; ++y)
        {
            scan_lines.emplace_back(ct_scan_single_line({0, static_cast<float>(y)}, 0., image));
        }
        
        // There are image_size_x * image_size_y variables.
        // If we want to solve a linear system, we need to have at least image_size_x * image_size_y
        // linearly independent equations.
        // For gradient desent, it dosen't matter. We alway have something in the end.
        const size_t num_scans = 0.3 * image_size_x * image_size_y;
        for(size_t iter = 0; iter < num_scans; ++iter)
        {
            constexpr float MARGIN = 5;
            const float x = (image_size_x - 2 * MARGIN) * (static_cast<float>(rand()) / (RAND_MAX)) + MARGIN;
            const float y = (image_size_x - 2 * MARGIN) * (static_cast<float>(rand()) / (RAND_MAX)) + MARGIN;
            // PI should cover the whole space
            const double theta = 2 * M_PI * (static_cast<double>(rand()) / (RAND_MAX));
            
            scan_lines.emplace_back(ct_scan_single_line({x, y}, theta, image));
        }

        return scan_lines;
    }

    // It is the same as: 
    //   Axb = scans_to_dense_Ab(...)
    //   NormalEquation = compute_normal_equation(Axb)
    // In this function, we compute the optimal condition directly. 
    // In other word, compute hessian directly. It is easy for CT since the f is simple.
    // e.g.   cost = sum(f_i)
    //   Since grad & hessian are linear operator (e.g. grad(x + x*x) = grad(x) + grad(x*x)),
    //   we just need to focus on f_i.
    //        f_i = ||a * x - b||^2
    //   grad:
    //        d(f_i) / d(x_i) = 2 * (a * x - b) * a_i 
    //        
    //   hessian:
    //        d(f_i) / d(xi*xj) = 2 * ai * aj
    //   hessian is diag
    //
    NormalEquation scans_to_normal_equation(const cv::Mat &variables,
                                            const std::vector<ScanLine> &scans)
    {
        const size_t num_equations = scans.size();
        NormalEquation normal_equation(num_variables_);

        for(size_t scan_idx = 0; scan_idx < num_equations; ++ scan_idx)
        {
            const auto &scan = scans[scan_idx];

            float a_times_x = 0.;
            for(const auto &var_i : scan.scaned_indexes)
            {
                const float a_i = 1.;
                a_times_x += variables.at<float>(var_i, 0) * a_i;
            }

            for(const auto &var_i : scan.scaned_indexes)
            {
                assert(var_i < static_cast<int>(num_variables_));
                const float a_i = 1.;
                for(const auto &var_j : scan.scaned_indexes)
                {
                    const float a_j = 1.;
                    normal_equation.lhs.at<float>(var_i, var_j) += 2. * a_i * a_j;
                }
                normal_equation.rhs.at<float>(var_i) -= 2. * (a_times_x - scan.accumulatedValue) * a_i;
            }
        }

        //l2_regularization(variables, normal_equation);
        l1_regularization(variables, normal_equation);

        return normal_equation;
    }

    void l2_regularization(const cv::Mat &variables,
                           NormalEquation &equation)
    {
        const double l2_weight = 1e-2 * num_variables_;
        const size_t num_vars = equation.lhs.rows;

        for(size_t var_idx = 0; var_idx < num_vars; ++var_idx)
        {
            equation.lhs.at<float>(var_idx, var_idx) += 2. * l2_weight;
            const float x_i = variables.at<float>(var_idx, 0);
            equation.rhs.at<float>(var_idx, 0) -= 2 * l2_weight * x_i;
        }
    }

    void l1_regularization(const cv::Mat &variables,
                           NormalEquation &equation)
    {
        const double l1_weight = 1e-2 * num_variables_;
        const size_t num_vars = equation.lhs.rows;

        for(size_t var_idx = 0; var_idx < num_vars; ++var_idx)
        {
            const float x_i = variables.at<float>(var_idx, 0);
            if(std::abs(x_i) < 1e-2)
            {
                // subgradient for x_i close to zero is whatever between [-1, 1]
            }
            else if(x_i > 0)
            {
                equation.rhs.at<float>(var_idx, 0) -= l1_weight;
            }
            else
            {
                equation.rhs.at<float>(var_idx, 0) -= -l1_weight;
            }
        }
    }

    void solve_normal_equation_dense(const cv::Mat &image,
                                     const NormalEquation &equation)
    {
        cv::Mat_<float> x;
        std::cout << "solving ..." << std::endl;
        const bool status = cv::solve(equation.lhs, equation.rhs, x, cv::DECOMP_CHOLESKY);
        if(status == false)
        {
            std::cerr << "failed to solve Ax=b by cholesky" << std::endl;
        }
        else
        {
            std::cout << "solved" << std::endl;
            show_variables({image.rows, image.cols}, x);
        }
    }

    cv::Mat solve_normal_equation_sparse(const cv::Mat &image,
                                      const NormalEquation &equation)
    {
        SparseSolver solver;
        cv::Mat delta = solver.solve_normal_eqution(equation);
        return delta;
    }

    void show_variables(const cv::Size im_size,
                        const cv::Mat variables,
                        const int wait_time = -1)
    {
        // wried func signature...
        cv::Mat im = variables.reshape(1, im_size.height);
        cv::imshow("variable", im);
        cv::waitKey(wait_time);
    }

    void debug_mat(const cv::Mat &m, std::string info = "mat")
    {
        std::cout << info << " size: " << m.rows << " " << m.cols << std::endl;
    }

    void newton(const cv::Mat &scaned_image,
                const std::vector<ScanLine> &scans)
    {
        cv::Mat variables = cv::Mat(num_variables_, 1, CV_32F, cv::Scalar(0));
        NormalEquation equation = scans_to_normal_equation(variables, scans);
        // solve_normal_equation_dense(scaned_image, equation);
        cv::Mat delta = solve_normal_equation_sparse(scaned_image, equation);
        variables += delta;
        show_variables({im_rows_, im_cols_}, variables);
    }

    void gradient_desent(const cv::Mat &scaned_image,
                         const std::vector<ScanLine> &scans)
    {
        cv::Mat variables = cv::Mat(num_variables_, 1, CV_32F, cv::Scalar(0));

        for(int iter = 0; iter < 1000; ++iter)
        {
            std::cout << "gd iter:" << iter << std::endl;
            const NormalEquation equation = scans_to_normal_equation(variables, scans);
            const cv::Mat &neg_grad = equation.rhs;
            variables += 0.1 / num_variables_ * neg_grad;
            show_variables({im_rows_, im_cols_}, variables, 1);
        }
    }

    void ct_reconstruction_simulation()
    {
        cv::Mat scaned_image = generate_small_im();
        std::vector<ScanLine> scans = ct_scan_random(scaned_image);

        if(true)
        {
            cv::imshow("scaned image", scaned_image);
            cv::waitKey(100);
        }

        gradient_desent(scaned_image, scans);
        // newton(scaned_image, scans);
    }

    int im_rows_ = 0;
    int im_cols_ = 0;
    int num_variables_ = 0;

};