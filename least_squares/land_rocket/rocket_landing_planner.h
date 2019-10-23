#pragma once

#include <vector>
#include <cmath>
#include <memory>

#include "types.h"
#include "residuals.h"
#include "least_square_problem.h"
#include "solver.h"


class RocketLandingPlanner
{
public:
    struct Config
    {
        Config()
        {
            weight_start << 1e-2, 1e3, 1e3, 1e3, 1e4, 1e-3, 1e-3;
            weight_end << 0., 1e3, 1e3, 1e3, 1e4, 1e3, 1e3; 
        }
        uint32_t iterations = 20;

        double update_step_size = 0.5;

        Vector7d weight_start;
        
        Vector7d weight_end;

        // "dense", "dense_in_place", "sparse"
        std::string solver_type = "sparse";
    };

    RocketLandingPlanner(const RocketState &start_state,
                         const RocketState &end_state,
                         const int num_state)
        :  start_state_(start_state), end_state_(end_state), num_states_(num_state)
    {
        initialize_trajectory();
    }

    // Least squares framework
    // 1. f(x) ~= J dx + b
    // 2. solve normal equation
    // 3. update x.
    void solve()
    {
        // print_variables();

        for(size_t i = 0; i < config_.iterations; ++i)
        {
            RocketLandingProblem problem;
            problem.residuals = compute_residaul(trajectory_, 
                start_state_, config_.weight_start, end_state_, config_.weight_end, num_states_);
            const VectorXd delta = solve_least_squares(problem.residuals);

            // update_variables(delta, config_.update_step_size);
            
            bool stop = back_tracking_line_search_and_update(problem.residuals, delta);

            if(stop)
            {
                std::cout << "Stop at iteration: " << i << std::endl;
                break;
            }

            // print_variables();
        }

        // print_variables();
    }

    void print_variables() const
    {
        for(int state_i = 0; state_i < num_states_; ++state_i)
        {
            std::cout << "state " << state_i << " :" 
                << trajectory_.states.at(state_i).variables.transpose() << std::endl;
        }
    }

    Trajectory get_trajectory() const
    {
        return trajectory_;
    }

protected:
    // initial guess for all variables
    void initialize_trajectory()
    {
        const Vector2d dp = end_state_.position() - start_state_.position();
        const Vector2d delta_p = dp / (num_states_ - 1.);
        
        const double heading_guess = atan2(dp.y(), dp.x());
        const double velocity_guess = 0.;
        const double accel_guess = 0.;
        const double turning_rate_guess = 0.;
        const double dt_guess = 1.;

        for(int i = 0; i < num_states_; ++i)
        {
            if(i == 0)
            {
                trajectory_.states.push_back(start_state_);
            }
            else if(i == num_states_ - 1)
            {
                trajectory_.states.push_back(end_state_);
            }
            else
            {
                RocketState state_i;
                state_i.delta_time() = dt_guess;
                state_i.position() = start_state_.position() + i * delta_p;
                state_i.velocity() = velocity_guess;
                state_i.acceleration() = accel_guess;
                state_i.turning_rate() = turning_rate_guess;
                state_i.heading() = heading_guess;

                trajectory_.states.push_back(state_i);
            }
        }
    }

    VectorXd solve_least_squares(const RocketLandingResiduals &residaul)
    {
        std::shared_ptr<RocketLandingSolver> solver_ptr;

        // "dense", "dense_in_place", "sparse"
        if(config_.solver_type == "dense")
        {
            solver_ptr = std::make_shared<DenseSolver>();
        }   
        else if(config_.solver_type == "dense_in_place")
        {
            solver_ptr = std::make_shared<DenseSolverUpdateInPlace>();
        }
        else if(config_.solver_type == "sparse")
        {
            solver_ptr = std::make_shared<SparseSolver>();
        }
        else
        {
            assert(false && "dude, solver not support");
        }

        VectorXd delta = solver_ptr->solver_rocket_landing_least_squares(residaul);
        
        return delta;
    }

    bool back_tracking_line_search_and_update(const RocketLandingResiduals current_residuals,
                                              const VectorXd &delta)
    {
        constexpr double BACK_TRACKING_SCALE = 0.5;

        const auto current_states = trajectory_.states;
        const double current_cost = current_residuals.total_cost();
        double updated_cost = current_cost + 1.;
        double final_k = 1;

        for(double k = 1.; k > 1e-2; k *= BACK_TRACKING_SCALE)
        {
            trajectory_.states = current_states;
            update_variables(delta, k);
            const RocketLandingResiduals updated_residuals = compute_residaul(trajectory_, 
                start_state_, config_.weight_start, end_state_, config_.weight_end, num_states_);
            updated_cost = updated_residuals.total_cost();

            if(updated_cost < current_cost)
            {
                final_k = k;
                break;
            }
        }

        std::cout << "current_cost :" << current_cost << std::endl;
        std::cout << "updated_cost :" << updated_cost << std::endl;
        std::cout << "back tracking k :" << final_k << std::endl;

        // TODO: better stopping condition
        // simple stoping condition
        if(current_cost - updated_cost < 1e-1)
        {
            return true;
        }
        else
        {
            return false;
        }
        
    }

    void update_variables(const VectorXd &delta, const double k)
    {
        for(int state_i = 0; state_i < num_states_; ++state_i)
        {
            const VectorXd delta_i = delta.segment(state_i * RocketState::STATE_SIZE, RocketState::STATE_SIZE);
            // std::cout << "i:" << state_i << "delta_i :" << delta_i.transpose() << std::endl;
            trajectory_.states.at(state_i).variables += k * delta_i;
        }
    }

    Config config_;

    // variables
    Trajectory trajectory_;

    // Prior data
    RocketState start_state_;
    RocketState end_state_;
    int num_states_ = -1;
};