#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>

#include "types.h"
#include "residuals.h"
#include "least_square_problem.h"


class RocketLandingSolver
{
public:
    struct LinearizedResidual
    {
        LinearizedResidual(const size_t num_equations, const size_t num_variables)
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

    virtual VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual) = 0;

protected:
    virtual VectorXd solve_normal_eqution(NormalEqution &quadratic) = 0;
};


// A solver for rocket landing problem
class DenseSolver : public RocketLandingSolver
{
public:
    virtual VectorXd solver_rocket_landing_least_squares(const RocketLandingResiduals &residual)
    {
        const LinearizedResidual linearized_residuals = linearized_residual_function(residual);
        NormalEqution normal_equ = linear_function_to_normal_equation(linearized_residuals);
        apply_regularization_to_hessian(residual, normal_equ);
        return solve_normal_eqution(normal_equ);
    }

protected:
    void add_residual(const Residual &residual,
                      int &residual_idx,
                      LinearizedResidual &equ)
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

    LinearizedResidual linearized_residual_function(const RocketLandingResiduals &residual)
    {
        if(verbose_) std::cout << "Constructing sparse system..." << std::endl;
        const int num_variables = residual.total_variable_size();
        const int num_equations = residual.total_residual_size();

        LinearizedResidual equ(num_equations, num_variables);

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
    void apply_regularization_to_hessian(
                        const RocketLandingResiduals &residual,
                        NormalEqution &normal_equation)
    {
        if(verbose_)  std::cout << "computing regularization..." << std::endl;

        const size_t num_variables = residual.total_variable_size();
        VectorXd reg_matrix_diag = VectorXd::Zero(num_variables);

        auto set_regularization_func = [&residual, &reg_matrix_diag, &normal_equation]
                                   (const double regularization, const int reg_idx)
        {
            for(int state_i = 0; state_i < residual.num_rocket_states; ++state_i)
            {
                const int var_idx = state_i * RocketState::STATE_SIZE + reg_idx;
                normal_equation.lhs(var_idx, var_idx) += 2 * regularization;
            }
        };

        if(residual.time_regularization > 0)
        {
            if(verbose_)  std::cout << "residual.time_regularization: " << residual.time_regularization << std::endl;
            set_regularization_func(residual.time_regularization, RocketState::i_dt);
        }

        if(residual.acceleration_regularization > 0)
        {
            if(verbose_)  std::cout << "residual.acceleration_regularization: " << residual.acceleration_regularization << std::endl;
            set_regularization_func(residual.acceleration_regularization, RocketState::i_acceleration);
        }

        if(residual.turning_rate_regularization > 0)
        {
            if(verbose_)  std::cout << "residual.turning_rate_regularization: " << residual.turning_rate_regularization << std::endl;
            set_regularization_func(residual.turning_rate_regularization, RocketState::i_turning_rate);
        }
    }

    // cost = ||Ax + b||_w 
    // cost = x.T A.T W A x - 2 * b.T W A x
    // dcost/dx = 2 * A.t * W * A * x + 2 * A.t * W.t * b
    // Normal equation:
    //        A.t * W * A * x = - A.t * W.t * b
    // Since most of W are sysmatic PSD
    //       A.t * W * A * x = - A.t * W * b
    NormalEqution linear_function_to_normal_equation(const LinearizedResidual &equ)
    {
        const int num_variables = equ.jacobi.cols();

        if(verbose_) std::cout << "computing lhs & rhs..." << std::endl;
        if(verbose_) std::cout << "Jacobian size: " << equ.jacobi.rows() 
            << "," << equ.jacobi.cols() << std::endl;

        if(verbose_) std::cout << "A * weight ..." << std::endl;
        // A.t * W, when W is diagnal
        MatrixXd At_times_W = equ.jacobi.transpose();
        for(int col = 0; col < At_times_W.cols(); ++col)
        {
            const double w = equ.weight(col, col);
            At_times_W.col(col) *= w;
        }

        NormalEqution normal_equ(num_variables);
        normal_equ.lhs = At_times_W * equ.jacobi;
        normal_equ.rhs = - At_times_W * equ.current_residaul;
        return normal_equ;
    }

    virtual VectorXd solve_normal_eqution(NormalEqution &quadratic)
    {
        auto &lhs = quadratic.lhs;
        auto &rhs = quadratic.rhs;

        std::cout << "solving Ax=b ..." << std::endl;
        VectorXd delta = (lhs).llt().solve(rhs);
        return delta;
    }

    bool verbose_ = false;
};
