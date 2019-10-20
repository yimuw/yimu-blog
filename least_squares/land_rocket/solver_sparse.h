#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/CholmodSupport>

#include "types.h"
#include "residuals.h"
#include "solver_update_ata_in_place.h"
#include "utils.h"


// A solver for rocket landing problem
class SparseSolver : public DenseSolverUpdateInPlace
{
public:
    virtual VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual) override
    {
        NormalEqution normal_equ = residual_function_to_quadratic(residual);
        apply_regularization(residual, normal_equ);
        return solve_normal_eqution(normal_equ);
    }

protected:
    virtual VectorXd solve_normal_eqution(NormalEqution &quadratic) override
    {
        return solve_normal_eqution_cholmod(quadratic);
    }

    VectorXd solve_normal_eqution_cholmod(NormalEqution &quadratic)
    {
        std::cout << "Creating SparseView..." << std::endl;
        // TODO: Can't find good doc for sparseView()
        const double reference = 1e-4;
        const double epsilon = 1e-6;
        Eigen::SparseMatrix<double> sparse_lhs = quadratic.lhs.sparseView(reference, epsilon);
        VectorXd &rhs = quadratic.rhs;

        Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> cholmod_solver;

        std::cout << "Cholmod Cholesky..." << std::endl;
        cholmod_solver.compute(sparse_lhs);

        if(cholmod_solver.info()!=Eigen::Success) 
        {
            assert(false && "Cholmod decompose failed");
        }

        std::cout << "Back sub..." << std::endl;
        Eigen::VectorXd delta = cholmod_solver.solve(rhs);
        if(cholmod_solver.info()!=Eigen::Success) 
        {
            assert(false && "Cholmod back sub failed");
        }

        return delta;
    }
};