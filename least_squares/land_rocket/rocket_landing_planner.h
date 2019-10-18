#pragma once

#include <vector>

#include "types.h"
#include "residuals.h"
#include "solver.h"


class RocketLandingPlanner
{
public:
    RocketLandingPlanner(const RocketState &start_state,
                         const RocketState &end_state,
                         const int num_state)
        :  start_state_(start_state), end_state_(end_state), num_states_(num_state)
    {
        initialize_trajectory();
    }

    // initial guess for all variables
    void initialize_trajectory()
    {
        const Vector2d dp = end_state_.position() - start_state_.position();
        const Vector2d delta_p = dp / num_states_;
        
        const double heading_guess = std::atan2(dp.y, dp.x);
        const double velocity_guess = 0.;
        const double accel_guess = 0.;
        const double turning_rate_guess = 0.;
        const double dt_guess = 1.;

        for(int i = 0; i < num_states_; ++i)
        {
            RocketState state_i;
            state_i.delta_time() = dt_guess;
            state_i.position() = start_state_.position() + i * delta_p;
            state_i.velocity() = velocity_guess;
            state_i.acceleration() = accel_guess;
            state_i.turning_rate() = turning_rate_guess;
            state_i.heading() = turning_rate_guess;

            trajectory_.states.push_back(state_i);
        }
    }

    RocketLandingResiduals compute_residaul()
    {
        RocketLandingResiduals residuals;

        assert(trajectory_.states.size() == num_states_);
        for(size_t i = 0; i < num_states_ - 1; ++i)
        {
            residuals.motion_residuals.emplace_back(trajectory_.states.at(i), i, 
                trajectory_.states.at(i + 1), i + 1);
        }

        residuals.start_state_prior = PriorResidual(trajectory_.states.at(0), 0, start_state_);
        residuals.end_state_prior = PriorResidual(trajectory_.states.at(num_states_ - 1), 0, start_state_);
        return residuals;
    }

    void solve_least_squares(const RocketLandingResiduals &residaul)
    {
        DenseSolver dense_solver;
        VectorXd delta = dense_solver.solver_rocket_landing_least_squares(residaul);
        update_variables(delta, 0.5);
    }

    void update_variables(const VectorXd &delta, const double k)
    {
        for(size_t state_i = 0; state_i < num_states_; ++state_i)
        {
            const VectorXd delta_i = delta.segment(state_i * RocketState::STATE_SIZE, RocketState::STATE_SIZE);
            trajectory_.states.at(state_i).variables += k * delta_i;
        }
    }

    // variables
    Trajectory trajectory_;

    // all residuals, it almost defines the optimization problem
    RocketLandingResiduals residuals_;

    RocketState start_state_;
    RocketState end_state_;
    int num_states_ = 100;
};