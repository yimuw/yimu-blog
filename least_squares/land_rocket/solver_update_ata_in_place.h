#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "types.h"
#include "residuals.h"
#include "utils.h"


// A solver for rocket landing problem
class DenseSolverUpdateInPlace : public DenseSolver
{
public:
    virtual VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual) override
    {
        ScopeProfiler p("DenseSolverUpdateInPlace:solver_rocket_landing_least_squares");

        NormalEquation normal_equ(residual.total_variable_size());
        residual_function_to_normal_equation(residual, normal_equ);
        apply_regularization_to_hessian(residual, normal_equ);
        return solve_normal_equation(normal_equ);
    }

protected:

    // TODO: template to make it faster
    // cost(x1, x2) = ||Ax1 + b||^2 + ||Ax2 + b||^2
    void add_residual_direct_update(const Residual &residual,
                                    int &residual_idx,
                                    NormalEquation &equ)
    {
        const MatrixXd jacobi = residual.jacobian();
        const MatrixXd weight = residual.weight();
        MatrixXd jtw = jacobi.transpose();
        if(residual.is_diagonal_weight())
        {
            for(int c = 0; c < jtw.cols(); ++c)
            {
                jtw.col(c) *= weight(c, c);
            }
        }
        else
        {
            jtw *= weight;
        }
        
        const int v_start_idx = residual.variable_start_index();
        const int v_size =  residual.variable_size();
        assert(v_start_idx + v_size <= equ.lhs.rows());
        assert(equ.lhs.rows() == equ.lhs.cols());
        
        equ.lhs.block(v_start_idx, v_start_idx, v_size, v_size) += jtw * jacobi;
        equ.rhs.segment(v_start_idx, v_size) += - jtw * residual.residual();

        if(false)
        {
            auto r = residual.residual();
            auto w = residual.weight();
            std::cout << "cost: " << r.transpose() * w * r << std::endl;
        }

        residual_idx += residual.residual_size();
    }

    void residual_function_to_normal_equation(const RocketLandingResiduals &residual,
                                                       NormalEquation &normal_equ)
    {
        ScopeProfiler p("residual_function_to_normal_equation");

        normal_equ.lhs.setZero();
        normal_equ.rhs.setZero();

        if(verbose_) std::cout << "Constructing sparse system..." << std::endl;
        
        // const int num_variables = residual.total_variable_size();
        const int num_equations = residual.total_residual_size();

        int residual_idx = 0;

        // Note: Order of residual matters!
        //       Handing order explicitly.
        //       We want A to be "close to" diagonal. 

        // start state
        add_residual_direct_update(residual.start_state_prior, residual_idx, normal_equ);
 
        // motions
        for(const auto &motion_r : residual.motion_residuals)
        {
            add_residual_direct_update(motion_r, residual_idx, normal_equ);
        }

        // end state
        add_residual_direct_update(residual.end_state_prior, residual_idx, normal_equ);

        assert(residual_idx == num_equations && "residual_idx messy up");
    }
};
