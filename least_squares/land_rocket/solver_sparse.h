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
        NormalEqution normal_equ;
        residual_function_to_normal_equation(residual, normal_equ);
        apply_regularization_to_hessian(residual, normal_equ);
        return solve_normal_eqution(normal_equ);
    }

protected:
    virtual VectorXd solve_normal_eqution(NormalEqution &normal_equ) override
    {
        ScopeProfiler p("solve_normal_equation");
#ifdef CHOLMOD_SUPPORT
        // Cholesky for A.T * A system
        return solve_normal_eqution_sparse<Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>>>(normal_equ);
#else
        return solve_normal_eqution_sparse<Eigen::SparseLU<Eigen::SparseMatrix<double>>>(normal_equ);
#endif
    }

    template<typename SOLVE_TYPE>
    VectorXd solve_normal_eqution_sparse(NormalEqution &normal_equ)
    {
        if(verbose_) std::cout << "Creating SparseView..." << std::endl;
        
        // TODO: Can't find good doc for sparseView()
        const double reference = 1e-4;
        const double epsilon = 1e-6;
        Eigen::SparseMatrix<double> sparse_lhs = normal_equ.lhs.sparseView(reference, epsilon);
        VectorXd &rhs = normal_equ.rhs;

        SOLVE_TYPE solver;

        if(verbose_) std::cout << "Decomposing..." << std::endl;
        solver.compute(sparse_lhs);

        if(solver.info()!=Eigen::Success) 
        {
            assert(false && "Decompose failed");
        }

        if(verbose_) std::cout << "Back sub..." << std::endl;
        Eigen::VectorXd delta = solver.solve(rhs);
        if(solver.info()!=Eigen::Success) 
        {
            assert(false && "Back sub failed");
        }

        return delta;
    }

    bool verbose_ = false;
};