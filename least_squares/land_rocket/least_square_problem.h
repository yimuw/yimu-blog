#pragma once

#include "types.h"
#include "utils.h"
#include "residuals.h"

// We can call it: (1) a quatratic cost function
//                 (2) a least-square problem
//                 (3) a factor graph
//                 (4) log of a gaussian distribution
//                 and whatever
struct RocketLandingResiduals
{
    int total_residual_size() const
    {
        return RocketState::STATE_SIZE * (motion_residuals.size() + 1 + 1);
    }

    int total_variable_size() const
    {
        return RocketState::STATE_SIZE * num_rocket_states;
    }

    double total_cost() const
    {
        auto const &residuals = *this;

        double total_cost = 0;

        auto regularization_cost = [&residuals](const RocketState &s)
        {
            double cost = 0.;

            const double dt1 = s.delta_time();
            const double vel1 = s.velocity();
            const double acc1 = s.acceleration();
            const double tr1 = s.turning_rate();

            cost += residuals.time_regularization * dt1 * dt1;
            cost += residuals.velocity_regularization * vel1 * vel1;
            cost += residuals.acceleration_regularization * acc1 * acc1;
            cost += residuals.turning_rate_regularization * tr1 * tr1;

            return cost;
        };

        for(size_t i = 0; i < residuals.motion_residuals.size(); ++i)
        {
            total_cost += residuals.motion_residuals.at(i).cost();

            total_cost += regularization_cost(residuals.motion_residuals.at(i).state1);
        }

        total_cost += regularization_cost(residuals.motion_residuals.back().state2);

        total_cost += residuals.start_state_prior.cost();
        total_cost += residuals.end_state_prior.cost();

        return total_cost;
    }
    
    // Using residual types explicitly.
    // Motivation is residual ordering matters in solving Ax=b.
    std::vector<MotionResidual> motion_residuals;
    PriorResidual start_state_prior;
    PriorResidual end_state_prior;

    int num_rocket_states = -1;

    // Regularization can be seem as a residual.
    // e.g r = k * x. in term of r = A x + b, A = k * I, b = 0.
    // But In this implementation, I handle regularization in A^T * A directly,
    // since it is simple & efficient.
    double time_regularization = -1;

    double velocity_regularization = -1;

    double acceleration_regularization = -1;

    double turning_rate_regularization = -1;
};

struct Constrains
{
    bool statisfy_constrains(const Trajectory &trajectory)
    {
        for(const auto &s : trajectory.states)
        {
            if(s.acceleration() < min_acceleration 
                or s.position().x() < min_position_x 
                or s.position().y() < min_position_y)
            {
                return false;
            }
        }
        return true;
    }

    // Rocket shoudn't hit ground
    double min_position_x = 0. - 1e-4;
    double min_position_y = 0. - 1e-4;

    // Acceleration is positive
    double min_acceleration = 0.;
};

RocketLandingResiduals compute_residaul(const Trajectory &trajectory,
                                        const RocketState &start_state,
                                        const Vector7d &weight_start,
                                        const RocketState &end_state,
                                        const Vector7d &weight_end,
                                        const int num_states)
{
    assert(num_states >= 2);
    RocketLandingResiduals residuals;

    assert(trajectory.states.size() == static_cast<size_t>(num_states));
    for(int i = 0; i < num_states - 1; ++i)
    {
        residuals.motion_residuals.emplace_back(trajectory.states.at(i), i, 
            trajectory.states.at(i + 1), i + 1, 1. / num_states);
    }

    residuals.start_state_prior = PriorResidual(trajectory.states.at(0), 0, 
        start_state, weight_start);

    residuals.end_state_prior = PriorResidual(trajectory.states.at(num_states - 1), 
        num_states - 1, end_state, weight_end);

    residuals.num_rocket_states = num_states;
    residuals.time_regularization = 0.1;
    residuals.velocity_regularization = 0.1;
    residuals.acceleration_regularization = 0.05;
    residuals.turning_rate_regularization = 0.05;

    return residuals;
}

// All data & operations solver needs
struct RocketLandingProblem
{
    // Residuals at current state. Need to be updated for new state.
    RocketLandingResiduals residuals;
    // Constrains
    Constrains constrains;
};