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
    RocketLandingPlanner(const RocketState &start_state,
                         const RocketState &end_state,
                         const int num_states)
        :  start_state_(start_state), end_state_(end_state)
    {
        config_.num_states = num_states;
        initialize_problem();
    }

    void solve()
    {
        // Primal Dual Interior Point handles iteration internally. 
        RocketLandingSolver_PrimalDualInteriorPoint solver;
        solver.solver_rocket_landing_constrained_least_squares(config_, problem_);
    }

    Trajectory get_trajectory() const
    {
        return problem_.trajectory;
    }

    // initial guess for all variables
    void initialize_problem()
    {
        problem_.config = config_;
        problem_.start_state = start_state_;
        problem_.end_state = end_state_;

        const Vector2d dp = end_state_.position() - start_state_.position();
        const Vector2d delta_p = dp / (config_.num_states - 1.);
        
        const double heading_guess = atan2(dp.y(), dp.x());
        const double accel_guess = 0.;
        const double turning_rate_guess = 0.;
        const double dt_guess = start_state_.delta_time();
        const Vector2d velocity_guess = delta_p / dt_guess;

        for(int i = 0; i < config_.num_states; ++i)
        {
            if(i == 0)
            {
                problem_.trajectory.states.push_back(start_state_);
            }
            else if(i == config_.num_states - 1)
            {
                problem_.trajectory.states.push_back(end_state_);
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

                problem_.trajectory.states.push_back(state_i);
            }
        }

        problem_.update_problem();
    }

    Config config_;

    // variables
    RocketLandingProblem problem_;

    // Prior data
    RocketState start_state_;
    RocketState end_state_;
};