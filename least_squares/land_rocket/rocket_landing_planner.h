#pragma once

#include <vector>
#include <cmath>
#include <memory>

#include "types.h"
#include "residuals.h"
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

    void solve()
    {
        // print_variables();

        for(size_t i = 0; i < config_.iterations; ++i)
        {
            const RocketLandingResiduals residuals = compute_residaul();
            const VectorXd delta = solve_least_squares(residuals);
            update_variables(delta, config_.update_step_size);

            // print_variables();
        }

        print_variables();
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

    RocketLandingResiduals compute_residaul()
    {
        assert(num_states_ >= 2);
        RocketLandingResiduals residuals;

        assert(trajectory_.states.size() == static_cast<size_t>(num_states_));
        for(int i = 0; i < num_states_ - 1; ++i)
        {
            residuals.motion_residuals.emplace_back(trajectory_.states.at(i), i, 
                trajectory_.states.at(i + 1), i + 1);
        }

        residuals.start_state_prior = PriorResidual(trajectory_.states.at(0), 0, 
            start_state_, config_.weight_start);
 
        residuals.end_state_prior = PriorResidual(trajectory_.states.at(num_states_ - 1), 
            num_states_ - 1, end_state_, config_.weight_end);

        residuals.num_rocket_states = num_states_;
        residuals.time_regularization = 0.1;
        residuals.velocity_regularization = 0.1;
        residuals.acceleration_regularization = 0.05;
        residuals.turning_rate_regularization = 0.05;

        return residuals;
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

    // all residuals, it almost defines the optimization problem
    RocketLandingResiduals residuals_;

    RocketState start_state_;
    RocketState end_state_;
    int num_states_ = -1;
};