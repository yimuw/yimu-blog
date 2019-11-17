#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

#ifdef CHOLMOD_SUPPORT
#include <eigen3/Eigen/CholmodSupport>
#endif

#include "ct_types.h"

using VectorX = Eigen::VectorXd;
using MatrixX = Eigen::MatrixXd;
using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

class SparseSolver
{
public:

    cv::Mat solve_normal_eqution(const NormalEquation &normal_equ)
    {
#ifdef CHOLMOD_SUPPORT
        // Cholesky for A.T * A system
        return solve_normal_eqution_sparse<Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>>(normal_equ);
#else
        return solve_normal_eqution_sparse<Eigen::SparseLU<Eigen::SparseMatrix<double>>>(normal_equ);
#endif
    }

private:
    template<typename SOLVE_TYPE>
    cv::Mat solve_normal_eqution_sparse(const NormalEquation &normal_equ)
    {
        std::cout << "Converting types..." << std::endl;
        // Eigen::SparseMatrix<double> sparse_lhs = normal_equ.lhs.sparseView(reference, epsilon);
        SparseMatrix sparse_lhs = mat_to_eigen_sparse(normal_equ.lhs);
        VectorX rhs = cv_mat_to_eigen(normal_equ.rhs);

        SOLVE_TYPE solver;

        std::cout << "Decomposing..." << std::endl;
        solver.compute(sparse_lhs);

        if(solver.info()!=Eigen::Success) 
        {
            assert(false && "Decompose failed");
        }

        std::cout << "Back sub..." << std::endl;
        VectorX delta = solver.solve(rhs);
        if(solver.info()!=Eigen::Success) 
        {
            assert(false && "Back sub failed");
        }
        std::cout << "Done" << std::endl;

        return vectorf_to_cv_mat(delta);
    }

    SparseMatrix mat_to_eigen_sparse(const cv::Mat &cv_mat)
    {
        std::vector<Triplet> tripletList;
        tripletList.reserve(100 * 100 * 100 * 100);
        for(int col = 0; col < cv_mat.cols; ++col)
        {
            for(int row = 0; row < cv_mat.rows; ++row)
            {
                const float value = cv_mat.at<float>(row, col);
                if(std::abs(value) > 1e-6)
                {
                    tripletList.push_back(Triplet(row, col ,value));
                }
            }
        }
        SparseMatrix sparse_mat(cv_mat.rows,cv_mat.cols);
        sparse_mat.setFromTriplets(tripletList.begin(), tripletList.end());
        sparse_mat.makeCompressed();

        return sparse_mat;
    }

    cv::Mat vectorf_to_cv_mat(const VectorX &eigen_vec)
    {
        cv::Mat cv_vec(eigen_vec.size(), 1, CV_32F, cv::Scalar(0.));
        for(int i = 0; i < eigen_vec.size(); ++i)
        {
            cv_vec.at<float>(i, 0) = eigen_vec[i];
        }
        return cv_vec;
    }

    VectorX cv_mat_to_eigen(const cv::Mat &cv_vec)
    {
        assert(cv_vec.cols == 1);
        VectorX eigen_vec(cv_vec.rows);
        for(int i = 0; i < eigen_vec.size(); ++i)
        {
            eigen_vec[i] = cv_vec.at<float>(i, 0);
        }
        return eigen_vec;
    }
};