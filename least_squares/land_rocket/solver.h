#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "types.h"
#include "residuals.h"

class DenseSolver
{
public:
    VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual)
    {
        LinearizedResiduals linearized_residuals = linearized_residual_function(residual);
        return solve_normal_eqution(linearized_residuals);
    }

private:
    struct LinearizedResiduals
    {
        LinearizedResiduals(const size_t num_equations, const size_t num_variables)
            : jacobi(MatrixXd(num_equations, num_variables)),
              current_residaul(VectorXd::Zero(num_equations))
        {
        }
        
        MatrixXd jacobi;
        // residaul evaluated at current x
        VectorXd current_residaul;
    };

    LinearizedResiduals linearized_residual_function(const RocketLandingResiduals &residual)
    {
        std::cout << "Constructing sparse system..." << std::endl;
        const size_t num_variables = residual.total_variable_size();
        const size_t num_equations = residual.total_residual_size();

        LinearizedResiduals equ(num_equations, num_variables);

        int residual_idx = 0;

        // Note: We can do polymorphism, but personal, I think it's only for GUI.
        //       Polymorphism makes code hard to understand, and hides intents.
        //       I prefer straightforward code.

        // Note: Order of residaul matters!
        //       We want A to be "close to" diagnal. 

        // start state
        {
            auto &start_r = residual.start_state_prior;
            equ.jacobi.block(residual_idx, start_r.state_index, 
                                start_r.residual_size(), start_r.variable_size()) 
                            = start_r.jacobian();
            equ.current_residaul.segment(residual_idx, start_r.residual_size()) = start_r.residual();
            residual_idx += start_r.residual_size();
        }

        // motions
        for(const auto &motion_r : residual.motion_residuals)
        {
            equ.jacobi.block(residual_idx, motion_r.state1_index, 
                               motion_r.residual_size(), motion_r.variable_size()) 
                        = motion_r.jacobian();
            equ.current_residaul.segment(residual_idx, motion_r.residual_size()) = motion_r.residual();

            residual_idx += motion_r.residual_size();
        }

        // end state
        {
            auto &end_r = residual.start_state_prior;
            equ.jacobi.block(residual_idx, end_r.state_index, 
                                end_r.residual_size(), end_r.variable_size()) 
                            = end_r.jacobian();
            equ.current_residaul.segment(residual_idx, end_r.residual_size()) = end_r.residual();
            residual_idx += end_r.residual_size();
        }

        return equ;
    }

    VectorXd solve_normal_eqution(const LinearizedResiduals &equ)
    {
        VectorXd delta = (equ.jacobi.transpose() * equ.jacobi).llt().solve(- equ.jacobi.transpose() * equ.current_residaul);
        return delta;
    }
};
