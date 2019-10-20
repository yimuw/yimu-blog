#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "types.h"
#include "residuals.h"


// A solver for rocket landing problem
class DenseSolver
{
public:
    VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual)
    {
        const LinearizedResiduals linearized_residuals = linearized_residual_function(residual);
        const VectorXd reg_matrix_diag = compute_regularization_matrix_diag(residual);
        return solve_normal_eqution(linearized_residuals, reg_matrix_diag);
    }

private:
    struct LinearizedResiduals
    {
        LinearizedResiduals(const size_t num_equations, const size_t num_variables)
            : jacobi(MatrixXd::Zero(num_equations, num_variables)),
              current_residaul(VectorXd::Zero(num_equations)),
              weight(MatrixXd::Zero(num_equations, num_equations))
        {
        }
        
        // jacobi of residual evaluated at currrent x
        MatrixXd jacobi;
        
        // residaul evaluated at current x
        VectorXd current_residaul;
        
        // weight matrix for the least square problem
        // e.g. x.T * A.T * W * A * x
        MatrixXd weight;
    };

    void add_residual(const Residual &residual,
                      int &residual_idx,
                      LinearizedResiduals &equ)
    {
        assert(residual.variable_start_index() + residual.variable_size() <= equ.jacobi.cols());
        assert(residual_idx + residual.residual_size() <= equ.jacobi.rows());
        equ.jacobi.block(residual_idx, residual.variable_start_index(), 
                            residual.residual_size(), residual.variable_size()) 
                        = residual.jacobian();

        equ.current_residaul.segment(residual_idx, residual.residual_size()) = residual.residual();
        equ.weight.block(residual_idx, residual_idx, residual.residual_size(), residual.residual_size())
            = residual.weight();

        if(false)
        {
            auto r = residual.residual();
            auto w = residual.weight();
            std::cout << "cost: " << r.transpose() * w * r << std::endl;
        }

        residual_idx += residual.residual_size();
    }

    LinearizedResiduals linearized_residual_function(const RocketLandingResiduals &residual)
    {
        std::cout << "Constructing sparse system..." << std::endl;
        const int num_variables = residual.total_variable_size();
        const int num_equations = residual.total_residual_size();

        LinearizedResiduals equ(num_equations, num_variables);

        int residual_idx = 0;

        // Note: Order of residaul matters!
        //       Handing order explicitly.
        //       We want A to be "close to" diagnal. 

        // start state
        add_residual(residual.start_state_prior, residual_idx, equ);
 
        // motions
        for(const auto &motion_r : residual.motion_residuals)
        {
            add_residual(motion_r, residual_idx, equ);
        }

        // end state
        add_residual(residual.end_state_prior, residual_idx, equ);

        assert(residual_idx == num_equations && "residual_idx messy up");

        return equ;
    }

    // basically, cost = ||Ax||^2 + k*||x||^2
    //            hessian_of_cost = A^*A + k*I
    VectorXd compute_regularization_matrix_diag(const RocketLandingResiduals &residual)
    {
        std::cout << "computing regularization..." << std::endl;

        const size_t num_variables = residual.total_variable_size();
        VectorXd reg_matrix_diag = VectorXd::Zero(num_variables);

        auto set_regularization = [&residual, &reg_matrix_diag](const double regularization,
                                     const int reg_idx)
        {
            for(int state_i = 0; state_i < residual.num_rocket_states; ++state_i)
            {
                reg_matrix_diag(state_i * RocketState::STATE_SIZE + reg_idx) 
                    = regularization;
            }
        };

        if(residual.time_regularization > 0)
        {
            std::cout << "residual.time_regularization: " << residual.time_regularization << std::endl;
            set_regularization(residual.time_regularization, RocketState::i_dt);
        }

        if(residual.velocity_regularization > 0)
        {
            std::cout << "residual.velocity_regularization: " << residual.velocity_regularization << std::endl;
            set_regularization(residual.velocity_regularization, RocketState::i_velocity);
        }

        return reg_matrix_diag;
    }

    // cost = ||Ax + b||_w 
    // cost = x.T A.T W A x - 2 * b.T W A x
    // dcost/dx = 2 * A.t * W * A * x + 2 * A.t * W.t * b
    // Normal equation:
    //        A.t * W * A * x = - A.t * W.t * b
    // Since most of W are sysmatic PSD
    //       A.t * W * A * x = - A.t * W * b
    VectorXd solve_normal_eqution(const LinearizedResiduals &equ, const VectorXd &regularization)
    {
        std::cout << "computing lhs & rhs..." << std::endl;
        std::cout << "Jacobian size: " << equ.jacobi.rows() 
            << "," << equ.jacobi.cols() << std::endl;

        std::cout << "A * weight ..." << std::endl;
        // A.t * W, when W is diagnal
        MatrixXd At_times_W = equ.jacobi.transpose();
        for(int col = 0; col < At_times_W.cols(); ++col)
        {
            const double w = equ.weight(col, col);
            At_times_W.col(col) *= w;
        }

        std::cout << "A.t * A..." << std::endl;
        MatrixXd lhs = At_times_W * equ.jacobi;

        // Handle regularization
        assert(lhs.cols() == regularization.rows());
        // std::cout << regularization << std::endl;
        for(int i = 0; i < lhs.cols(); ++i)
        {
            lhs(i, i) += regularization[i];
        }

        std::cout << "A.t * b..." << std::endl;
        const MatrixXd rhs = - At_times_W * equ.current_residaul;

        std::cout << "solving Ax=b ..." << std::endl;
        VectorXd delta = (lhs).llt().solve(rhs);
        return delta;
    }
};
