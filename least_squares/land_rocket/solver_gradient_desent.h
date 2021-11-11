#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "types.h"
#include "residuals.h"
#include "least_square_problem.h"
#include "solver_update_ata_in_place.h"


// A (hacky) gradient desent solver for rocket landing problem
class GradientDesentSolver : public DenseSolverUpdateInPlace
{
public:
    virtual VectorXd solver_rocket_landing_least_squares_single_step(const RocketLandingResiduals &residual)
    {
        std::cerr << "GradientDesentSolver didn't impl ls" << std::endl;
        return {};
    }

    virtual void solver_rocket_landing_least_squares(Config& config,
                                                     RocketLandingProblem& problem)
    {
        constexpr int MAX_ITERATIONS = 1000000;
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            ScopeProfiler p("solve_iter");

            problem.update_problem();

            const VectorXd step = compute_gradient(trajectory_to_vectorXd(problem.trajectory), problem.residuals);
            problem.trajectory = update_primal_variables(step, -0.00005, problem.trajectory);

            std::cout << "step: " << step.norm() << std::endl;
            
            if (step.norm() < 1e-4) 
            {
                std::cout << "stop at iter:" << iter << std::endl;
                break;
            }
        }
    }

protected:

    VectorXd compute_gradient(const VectorXd &x_cur, const RocketLandingResiduals &residual)
    {
        NormalEquation normal_equ(residual.total_variable_size());
        residual_function_to_normal_equation(residual, normal_equ);
        apply_regularization_to_hessian(residual, normal_equ);

        VectorXd grad = normal_equ.lhs * x_cur - normal_equ.rhs;
        return grad;
    }
};
