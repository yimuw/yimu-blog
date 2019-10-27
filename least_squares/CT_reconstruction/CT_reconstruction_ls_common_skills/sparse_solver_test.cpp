/* ========================================================================== */
/* === Demo/cholmod_simple ================================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Demo Module.  Copyright (C) 2005-2006, Timothy A. Davis
 * -------------------------------------------------------------------------- */

/* Read in a real symmetric or complex Hermitian matrix from stdin in
 * MatrixMarket format, solve Ax=b where b=[1 1 ... 1]', and print the residual.
 * Usage: cholmod_simple < matrixfile
 */
#include <iostream>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/CholmodSupport>


Eigen::MatrixXd generate_A()
{
    Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(10, 10);
    for(size_t rows = 0; rows < 10; ++rows)
    {
        for(size_t cols = 0; cols < 10; ++ cols)
        {
            mat(rows, cols) = (double) rand() / RAND_MAX/ 20;
        }
    }

    // mat = Eigen::MatrixXd::Identity(10, 10);
    
    return mat;
}

Eigen::SparseMatrix<double> convert_to_sparse(Eigen::MatrixXd &mat)
{
    Eigen::SparseMatrix<double> sparse_mat = mat.sparseView();
    return sparse_mat;
}


int main (void)
{
    Eigen::MatrixXd A = generate_A();

    Eigen::MatrixXd dense_lhs = A.transpose() * A;

    Eigen::SparseMatrix<double> lhs = convert_to_sparse(dense_lhs);
    std::cout << "dense lhs:" << dense_lhs << std::endl;
    Eigen::VectorXd rhs = Eigen::VectorXd::Ones(10);

    Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(lhs);
    if(solver.info()!=Eigen::Success) {
    // decomposition failed
    }
    Eigen::MatrixXd x = solver.solve(rhs);
    if(solver.info()!=Eigen::Success) {
    }

    std::cout << "x: " << x << std::endl;



    return (0) ;
}