#pragma once

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/CholmodSupport>

#include "ct_types.h"


class SparseSolve
{
    struct SparseEqution
    {
        SparseEqution(const size_t num_equations, const size_t num_variables)
            : A(Eigen::SparseMatrix<double>(num_equations, num_variables)),
              b(Eigen::VectorXd::Zero(num_equations))
        {
        }
        
        Eigen::SparseMatrix<double> A;
        Eigen::VectorXd b;
    };

    SparseEqution scans_to_sparse_Ab(const cv::Mat &image,
                                     const std::vector<ScanLine> &scans)
    {
        std::cout << "Constructing sparse system..." << std::endl;
        const size_t num_variables = image.cols * image.rows;
        const size_t num_equations = scans.size();

        SparseEqution sparse_equ(num_equations, num_variables);

        using triplet = Eigen::Triplet<double>;
        std::vector<triplet> triplets;
        
        for(size_t scan_idx = 0; scan_idx < num_equations; ++ scan_idx)
        {
            const auto &scan = scans[scan_idx];
            sparse_equ.b(scan_idx, 0) = scan.accumulatedValue;

            for(const auto &var_idx : scan.scaned_indexes)
            {
                assert(var_idx < num_variables);

                triplets.emplace_back(scan_idx, var_idx, 1.);
                // equation.A.at<float>(scan_idx, var_idx) = 1.;
            }
        }

        sparse_equ.A.setFromTriplets(triplets.begin(), triplets.end());

        return sparse_equ;
    }

    Eigen::VectorXd solve_normal_equation_sparse(const SparseEqution &sparse_equ)
    {
        std::cout << "Constructing normal euqations..." << std::endl;
        const Eigen::SparseMatrix<double> lhs = sparse_equ.A.transpose() * sparse_equ.A;

        std::cout << "lhs non zeros: " << lhs.nonZeros() 
            << " lhs size:" << lhs.rows() * lhs.cols() << std::endl;
        const Eigen::VectorXd rhs = sparse_equ.A.transpose() * sparse_equ.b;

        Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver;

        std::cout << "Cholesky..." << std::endl;
        solver.compute(lhs);

        if(solver.info()!=Eigen::Success) 
        {
            std::cerr << "Cholmod decompose failed" << std::endl;
        }

        std::cout << "Back sub..." << std::endl;
        Eigen::VectorXd x = solver.solve(rhs);
        if(solver.info()!=Eigen::Success) 
        {
            std::cerr << "Cholmod solve failed" << std::endl;
        }

        return x;
    }

    cv::Mat convert_to_im(const Eigen::VectorXd &x, 
                          const size_t rows, 
                          const size_t cols)
    {
        cv::Mat image(rows,cols, CV_32F, cv::Scalar(0.));
        for(size_t r = 0; r < rows; ++r)
        {
            for(size_t c = 0; c < cols; ++c)
            {
                image.at<float>(r,c) = x(r*cols + c);
            }
        }
        
        return image;
    }

public:
    cv::Mat reconstrcut_ct_sparse_solver(const cv::Mat &image,
                                         const std::vector<ScanLine> &scans)
    {
        SparseEqution sparse_equ = scans_to_sparse_Ab(image, scans);
        Eigen::VectorXd result = solve_normal_equation_sparse(sparse_equ);
        cv::Mat image_reconstructed = convert_to_im(result, image.rows, image.cols);

        cv::imshow("image_reconstructed sparse", image_reconstructed);
        cv::waitKey(-1);

        return image;
    }
};
