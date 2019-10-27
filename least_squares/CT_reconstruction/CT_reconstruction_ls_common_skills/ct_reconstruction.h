#pragma once

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ct_types.h"
#include "sparse_solver.h"


class CtReconstruction
{
public:
    
    cv::Mat generate_small_im()
    {
        constexpr int im_size = 100;

        cv::Mat image(im_size,im_size, CV_32F, cv::Scalar(0.));

        {
            cv::Rect region_of_interest = cv::Rect(im_size / 2, im_size / 2, im_size / 4, im_size / 4);
            cv::Mat image_roi = image(region_of_interest);
            image_roi.setTo(1.);
        }
        {
            cv::Rect region_of_interest = cv::Rect(0, 0, im_size / 4, im_size / 4);
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
                // std::cout << " xy:" << x <<"," << y << " v: " << image.at<float>(y, x) << std::endl;

                scan_line.scaned_indexes.push_back(index_map(image, x, y));
                scan_line.accumulatedValue += image.at<float>(y, x);
            }
        };

        // It is a classical computer graphic problem.
        // I try to solve the problem when a = 999, loop through x only pass 1 pixel.
        // Loop x
        constexpr double HALF_SPACE_A = 0.5;

        // std::cout << "center: " << center << " a b theta " << a << "," << b << "," << theta << "\n";

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
        
        const size_t num_scans = 1.01 * image_size_x * image_size_y;
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

    DenseEquation scans_to_dense_Ab(const cv::Mat &image,
                                    const std::vector<ScanLine> &scans)
    {
        const size_t num_variables = image.cols * image.rows;
        const size_t num_equations = scans.size();

        DenseEquation equation(num_equations, num_variables);

        for(size_t scan_idx = 0; scan_idx < num_equations; ++ scan_idx)
        {
            const auto &scan = scans[scan_idx];
            equation.b.at<float>(scan_idx, 0) = scan.accumulatedValue;

            for(const auto &var_idx : scan.scaned_indexes)
            {
                assert(var_idx < num_variables);
                equation.A.at<float>(scan_idx, var_idx) = 1.;
            }
        }

        return equation;
    }

    void solve_normal_equation_dense(const cv::Mat &image,
                                     const DenseEquation &equation)
    {
        const size_t num_variables = image.cols * image.rows;
        const size_t num_equations = equation.b.rows;
        // want to minimize f = 1/n * (Ax - b)^2
        // df / dx = 2/n * (Ax - b) * A.T
        const cv::Mat &A = equation.A;
        const cv::Mat &b = equation.b;

        cv::Mat_<float> x;
        cv::solve(A, b, x, cv::DECOMP_CHOLESKY | cv::DECOMP_NORMAL);

        show_variables({image.rows, image.cols}, x, -1);
    }

    void show_variables(const cv::Size im_size,
                        const cv::Mat variables,
                        const int wait_time = 1)
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

    void gradient_desent(const cv::Mat &image,
                         const DenseEquation &equation)
    {
        const size_t num_variables = image.cols * image.rows;
        const size_t num_equations = equation.b.rows;
        // want to minimize f = 1/n * (Ax - b)^2
        // df / dx = 2/n * (Ax - b) * A.T
        const cv::Mat &A = equation.A;
        const cv::Mat &b = equation.b;
        cv::Mat x = cv::Mat(num_variables, 1, CV_32F, cv::Scalar(0.));

        constexpr size_t MAX_ITERS = 1000;
        constexpr double STEP_K = 0.1;
        for(size_t iter = 0; iter < MAX_ITERS; ++iter)
        {
            std::cout << "iter: " << iter << std::endl;
            const cv::Mat grad = 2. / num_equations * (A*x - b).t() * A;
            // debug_mat(grad, "grad");

            x -= STEP_K * grad.t();

            // std::cout << "grad:" << grad << std::endl;

            show_variables({image.rows, image.cols}, x);
        }
    }

    void ct_reconstruction_simulation()
    {
        cv::Mat scaned_image = generate_small_im();

        if(true)
        {
            cv::imshow("scaned image", scaned_image);
            cv::waitKey(100);
        }

        std::vector<ScanLine> scans = ct_scan_random(scaned_image);

        if(params_.optimizer_type == "gradient_desent")
        {
            DenseEquation equation = scans_to_dense_Ab(scaned_image, scans);
            gradient_desent(scaned_image, equation);
        }
        else if(params_.optimizer_type == "least_square_dense")
        {
            DenseEquation equation = scans_to_dense_Ab(scaned_image, scans);
            solve_normal_equation_dense(scaned_image, equation);
        }
        else if(params_.optimizer_type == "least_square_sparse")
        {
            SparseSolve sparse_solver;
            sparse_solver.reconstrcut_ct_sparse_solver(scaned_image, scans);
        }

    }

    struct Params
    {
        // "gradient_desent", "least_square_dense", "least_square_sparse"
        std::string optimizer_type = "least_square_dense";
    };

    Params params_;
};