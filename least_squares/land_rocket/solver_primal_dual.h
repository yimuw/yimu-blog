#pragma once

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include<Eigen/SparseLU>


#include "types.h"
#include "residuals.h"
#include "least_square_problem.h"
#include "solver_sparse.h"


// Inherient SparseSolver for code reuse and tests
class RocketLandingSolver_PrimalDualInteriorPoint : public SparseSolver
{
    // Note: primal variables is embeded in problem.trajectory
    //       Need to sleep, don't want to refactor it.
    using PrimalVariables = Trajectory;
public:
    void solver_rocket_landing_constrained_least_squares(const Config &config,
                                                         RocketLandingProblem &problem)
    {
        config_ = config;

        num_states_ = problem.trajectory.states.size();
        num_primal_variables_ = problem.trajectory.states.size() * RocketState::STATE_SIZE;
        num_dual_variables_ = problem.constrains.linear_constrains.size();
        const int augmented_var_size =  num_primal_variables_ + num_dual_variables_;

        // avoid memory reallocation
        NormalEqution augmented_normal_equ(augmented_var_size);
        NormalEqution normal_equ(num_primal_variables_);

        VectorXd dual_vars = initialize_dual_variables();
        assert(dual_vars.size() == num_dual_variables_);

        // print_variables(problem.trajectory, true);

        constexpr int MAX_ITERATIONS = 20;
        for(int iter = 0; iter < MAX_ITERATIONS; ++iter)
        {
            ScopeProfiler p("solve_iter");

            problem.update_problem();

            // unconstrained problem
            residual_function_to_normal_equation(problem.residuals, normal_equ);
            
            // compute the gradient of the cost function
            VectorXd cost_grad = compute_cost_gradient(normal_equ, problem.trajectory, problem.residuals);
            apply_regularization_to_hessian(problem.residuals, normal_equ);

            // Primal dual problem
            // Solve (Stationarity, Complementary slackness) of KKT
            const double k_relax_complementary_slackness =compute_k_relax_complementary_slackness(iter, problem, dual_vars);
            construct_primal_dual_problem(normal_equ.lhs, cost_grad, 
                dual_vars, problem.constrains, k_relax_complementary_slackness, augmented_normal_equ);
            
            // solve linear system
            VectorXd delta_primal_daul = solve_normal_eqution(augmented_normal_equ);
            assert(delta_primal_daul.size() == num_primal_variables_ + num_dual_variables_);

            // Enforce interior point 
            // (primal feasibility + dual feasibility) of KKT
            bool status = back_tracking_line_search_and_update(delta_primal_daul, problem, dual_vars);
            if(status == false)
            {
                std::cerr << "update failed at iter: " << iter << std::endl;
                std::cout << "update failed at iter: " << iter << std::endl;
                break;
            }

            if(false)
            {
                print_variables(problem.trajectory);
                PRINT_NAME_VAR(problem.residuals.total_cost());
                PRINT_NAME_VAR(problem.residuals.prior_cost());
                PRINT_NAME_VAR(problem.residuals.motion_cost());
                PRINT_NAME_VAR(problem.residuals.regularization_cost());
            }

            if(stop_condition(augmented_normal_equ.rhs, problem.constrains, dual_vars) == true)
            {
                std::cout << "stop at iteration:" << iter << std::endl;
                break;
            }
        }

        // print_variables(problem.trajectory, true);
    }
protected:
    double compute_k_relax_complementary_slackness(const size_t iteration, 
                                                   const RocketLandingProblem &problem, 
                                                   const VectorXd &dual_variables) const
    {
        // Trick
        // Using log barrier for some iterations
        // Smaller t => smoother cost 
        if(iteration < 5)
        {
            return 1.;
        }
        
        const double surrogate_duality_gap = compute_surrogate_duality_gap(problem.constrains, dual_variables);
        const double k_relax_complementary_slackness = primal_dual_mu_ * num_dual_variables_ / surrogate_duality_gap;
        
        // PRINT_NAME_VAR(k_relax_complementary_slackness);

        return k_relax_complementary_slackness;
    }

    VectorXd initialize_dual_variables()
    {
        return 1e1 * VectorXd::Ones(num_dual_variables_);
    }

    VectorXd compute_cost_gradient(const NormalEqution &normal_equ,
                                   const PrimalVariables &trajectory,
                                   const RocketLandingResiduals &residuals)
    {
        const VectorXd primal_vars = trajectory_to_vectorXd(trajectory);

        // cost = ||r||^2 + ||x||^2_w (W is ignored for residual)
        // cost gradient = 2 * Dr * r + wx,   Dr * r = A.T * b
        VectorXd cost_gredient = - 2. * normal_equ.rhs;
        assert(cost_gredient.rows() == num_primal_variables_);

        auto set_regularization_func = [&]
                            (const double regularization, 
                            const int reg_idx)
        {
            for(int state_i = 0; state_i < residuals.num_rocket_states; ++state_i)
            {
                const int var_idx = state_i * RocketState::STATE_SIZE + reg_idx;
                cost_gredient(var_idx) += 2 * regularization * primal_vars(var_idx);
            }
        };

        if(residuals.time_regularization > 0)
        {
            set_regularization_func(residuals.time_regularization, RocketState::i_dt);
        }

        if(residuals.acceleration_regularization > 0)
        {
            set_regularization_func(residuals.acceleration_regularization, RocketState::i_acceleration);
        }

        if(residuals.turning_rate_regularization > 0)
        {
            set_regularization_func(residuals.turning_rate_regularization, RocketState::i_turning_rate);
        }

        return cost_gredient;
    }

    // Solve for KKT: (Stationarity, Complementary slackness)
    void construct_primal_dual_problem(const MatrixXd &cost_hessian,
                                                const VectorXd &cost_gradient,
                                                const VectorXd &dual_variables, 
                                                const Constrains &constrains,
                                                const double k_relax_complementary_slackness,
                                                NormalEqution &augmented_normal_equ)
    {
        ScopeProfiler p("construct_primal_dual_problem");

        construct_primal_dual_problem_lhs(
            cost_hessian, dual_variables, constrains, augmented_normal_equ.lhs);

        augmented_normal_equ.rhs.setZero();
        augmented_normal_equ.rhs = construct_primal_dual_problem_rhs(
            cost_gradient, dual_variables, constrains, k_relax_complementary_slackness);
    }

        // Solve for KKT: (Stationarity, Complementary slackness)
    void construct_primal_dual_problem_lhs(const MatrixXd &cost_hessian,
                                           const VectorXd &dual_variables, 
                                           const Constrains &constrains,
                                           MatrixXd &augmented_normal_equ_lhs)
    {
        const int num_variables = cost_hessian.rows();
        augmented_normal_equ_lhs.setZero();

        // Copy primal equation
        augmented_normal_equ_lhs.block(0, 0, num_variables, num_variables) = cost_hessian;

        // Not general, simplified for 1d linear case.
        for(size_t constrain_idx = 0; constrain_idx < constrains.linear_constrains.size(); ++constrain_idx)
        {
            const auto &constrain_linear = constrains.linear_constrains[constrain_idx];
            const int correspond_primal_index = constrain_linear.state_index * RocketState::STATE_SIZE 
                + constrain_linear.type_index;
            const int dual_var_index = num_variables + constrain_idx;

            // upper left
            // hessian for linear constrain is 0. Don't need to update it

            // upper right block
            augmented_normal_equ_lhs(correspond_primal_index, dual_var_index) = constrain_linear.jacobian();
            // lower left block
            augmented_normal_equ_lhs(dual_var_index, correspond_primal_index) 
                = - dual_variables[constrain_idx] * constrain_linear.jacobian();
            // lower right block
            augmented_normal_equ_lhs(dual_var_index, dual_var_index) = - constrain_linear.h();
        }
    }

    VectorXd construct_primal_dual_problem_rhs(const VectorXd &cost_gradient,
                                            const VectorXd &dual_variables, 
                                            const Constrains &constrains,
                                            const double k_relax_complementary_slackness)
    {
        const int num_variables = cost_gradient.rows();
        const int num_constrains = constrains.linear_constrains.size();
        const int augmented_var_size =  num_variables + num_constrains;
        VectorXd relaxed_kkt_rhs = VectorXd::Zero(augmented_var_size);

        // relaxed_kkt_rhs.block(0, 0, num_variables, 1) = normal_equ.rhs;
        relaxed_kkt_rhs.segment(0, num_variables) = -cost_gradient;
        
        // Not general, simplified for 1d linear case.
        for(size_t constrain_idx = 0; constrain_idx < constrains.linear_constrains.size(); ++constrain_idx)
        {
            const auto &constrain_linear = constrains.linear_constrains[constrain_idx];
            const int correspond_primal_index = constrain_linear.state_index * RocketState::STATE_SIZE 
                + constrain_linear.type_index;
            const int dual_var_index = num_variables + constrain_idx;

            // Dual residual: KKT Stationarity
            relaxed_kkt_rhs(correspond_primal_index) -=  dual_variables[constrain_idx] * constrain_linear.jacobian();
            // Cent residual: KKT relaxed complementary slackness
            relaxed_kkt_rhs(dual_var_index) -= - dual_variables[constrain_idx] * constrain_linear.h() 
                - 1. / k_relax_complementary_slackness;
        }

        return relaxed_kkt_rhs;
    }

    bool dual_feasible(const VectorXd &dual_variables,
                       const VectorXd &dual_step,
                       const double step_size)
    {
        const VectorXd updated_dual_var = dual_variables + step_size * dual_step;
        // u_i > 0 for all i,
        if((updated_dual_var.array() >= 0.).all())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool primal_feasible(Constrains &constrains,
                         const PrimalVariables &trajectory)
    {
        return constrains.trajectory_statisfy_constrains(trajectory);
    }

    bool back_tracking_line_search_and_update(const VectorXd &delta_primal_daul,
                                              RocketLandingProblem &problem,
                                              VectorXd &dual_variables)
    {
        const double current_cost = problem.residuals.total_cost();
        double updated_cost = std::numeric_limits<double>::max();

        auto check_cost_deacrease = [&](const PrimalVariables &trajectory)
        {
            const RocketLandingResiduals updated_residuals = compute_residaul(trajectory, 
                problem.start_state, config_.weight_start, problem.end_state, config_.weight_end, num_states_);
            
            // TODO: I should use primal-dual cost here. But it is expensive.
            updated_cost = updated_residuals.total_cost();
            
            // Back tracking for value decrase
            if(updated_cost < current_cost)
            {
                return true;
            }
            else
            {
                return false;
            }
        };

        double final_line_search_step = 0.;
        bool search_success = false;
        // Cache
        bool dual_feasible_status = false;
        bool primal_feasible_status = false;

        const VectorXd primal_step = delta_primal_daul.segment(0, num_primal_variables_);
        const VectorXd dual_step = delta_primal_daul.segment(num_primal_variables_, num_dual_variables_);

        for(double step_size = 1.; step_size > 1e-2; step_size *= back_tracking_scale_)
        {
            if(dual_feasible_status == false)
            {
                if(dual_feasible(dual_variables, dual_step, step_size) == false)
                {
                    continue;
                }
                else
                {
                    dual_feasible_status = true;
                }
            }
            
            PrimalVariables updated_primal_vars = update_primal_variables(primal_step, step_size, problem.trajectory);

            // TODO: update check_cost_deacrease to use primal-dual cost
            final_line_search_step = step_size;

            if(primal_feasible_status == false)
            {
                if(primal_feasible(problem.constrains, updated_primal_vars) == false)
                {
                    continue;
                }
                else
                {
                    primal_feasible_status = true;
                }
            }
            
            if(check_cost_deacrease(updated_primal_vars) == false)
            {
                continue;
            }
            else
            {
                search_success = true;
                final_line_search_step = step_size;
                break;
            }
        }

        // update dual & primal vars
        dual_variables += final_line_search_step * dual_step;
        problem.trajectory = update_primal_variables(primal_step, final_line_search_step, problem.trajectory);

        // PRINT_NAME_VAR(dual_feasible_status);
        // PRINT_NAME_VAR(primal_feasible_status);
        // PRINT_NAME_VAR(search_success);
        // PRINT_NAME_VAR(final_line_search_step);

        return true;
    }

    double compute_surrogate_duality_gap(const Constrains &constrains,
                                         const VectorXd &dual_variables) const
    {
        double gap_value = 0.;

        for(size_t constrain_idx = 0; constrain_idx < constrains.linear_constrains.size(); ++constrain_idx)
        {
            const LinearConstrain1D &constrain_linear = constrains.linear_constrains[constrain_idx];
            gap_value += - dual_variables[constrain_idx] * constrain_linear.h();
        }
        return gap_value;
    }

    // Simple stop condition
    bool stop_condition(const VectorXd &residual,
                        const Constrains &constrains,
                        const VectorXd &dual_variables)
    {
        const double primal_dual_cost = residual.transpose() * residual;
        const double surrogate_duality_gap = compute_surrogate_duality_gap(constrains, dual_variables);
        // PRINT_NAME_VAR(surrogate_duality_gap);
        // PRINT_NAME_VAR(primal_dual_cost);
        if(primal_dual_cost > 1e-2)
        {
            return false;
        }

        if(surrogate_duality_gap > 1e-2)
        {
            return false;
        }

        return true;
    }


    Config config_;

    // Config
    double back_tracking_scale_ = {0.5};
    double primal_dual_mu_ = 10.;

    int num_states_ = {-1};
    int num_primal_variables_ = {-1};
    int num_dual_variables_ = {-1};
};
