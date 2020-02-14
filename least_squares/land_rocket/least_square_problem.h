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

    double prior_cost() const
    {
        return start_state_prior.cost() + end_state_prior.cost();
    }

    double motion_cost() const
    {
        double mcost = 0;
        for(size_t i = 0; i < motion_residuals.size(); ++i)
        {
            mcost += motion_residuals.at(i).cost();
        }
        return mcost;
    }

    double regularization_cost() const
    {
        double rcost = 0.;
        
        const auto &residuals = *this;
        auto regularization_cost = [residuals](const RocketState &s)
        {
            double cost = 0.;

            const double dt1 = s.delta_time();
            const double acc1 = s.acceleration();
            const double tr1 = s.turning_rate();

            if(residuals.time_regularization > 0)
                cost += residuals.time_regularization * dt1 * dt1;
            if(residuals.acceleration_regularization > 0)
                cost += residuals.acceleration_regularization * acc1 * acc1;
            if(residuals.turning_rate_regularization > 0)
                cost += residuals.turning_rate_regularization * tr1 * tr1;

            return cost;
        };

        for(size_t i = 0; i < motion_residuals.size(); ++i)
        {
            rcost += regularization_cost(motion_residuals.at(i).state1);
        }
        rcost += regularization_cost(motion_residuals.back().state2);

        return rcost;
    }

    double total_cost() const
    {
        return prior_cost() + motion_cost() + regularization_cost();
    }

    Trajectory get_variables() const
    {
        Trajectory traj;
        for(const auto &m_residual : motion_residuals)
        {
            traj.states.push_back(m_residual.state1);
        }
        traj.states.push_back(motion_residuals.back().state2);
        assert(static_cast<int>(traj.states.size()) == num_rocket_states);
        return traj;
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
    double time_regularization = 0.;

    double acceleration_regularization = 0.;

    double turning_rate_regularization = 0.;
};


RocketLandingResiduals compute_residaul(const Trajectory &trajectory,
                                        const RocketState &start_state,
                                        const Vector8d &weight_start,
                                        const RocketState &end_state,
                                        const Vector8d &weight_end,
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
    residuals.time_regularization = 0.;
    residuals.acceleration_regularization = 1e-1;
    residuals.turning_rate_regularization = 1e0;

    return residuals;
}


// General constain def:
//  h(x) <= 0

// k <= x
// h(x) = - x + k <= 0
struct LinearConstrain1D
{
    LinearConstrain1D(const int state_idx,
                      const int type_idx,
                      const int dual_idx,
                      const double xin, 
                      const double kin)
        : state_index(state_idx), type_index(type_idx), dual_index(dual_idx), x(xin), k(kin)
    {
    }

    double hessian() const
    {
        return 0.;
    }

    double jacobian() const
    {
        return - 1.;
    } 

    double h() const
    {
        return - x + k;
    }

    int state_index = {-1};
    int type_index = {-1};
    int dual_index = {-1};
    double x = 0.;
    double k = 0.;
};


struct Constrains
{
    bool trajectory_statisfy_constrains(const Trajectory &trajectory) const
    {
        for(const auto &s : trajectory.states)
        {
            if(s.acceleration() < min_acceleration 
                or s.delta_time() < min_dt
                or s.position().y() < min_position_y
                )
            {
                return false;
            }
        }
        return true;
    }

    void update_constrains(const Trajectory &trajectory)
    {
        linear_constrains.clear();
        linear_constrains.reserve(3 * trajectory.states.size());
        for(size_t si = 0; si < trajectory.states.size(); ++si)
        {
            const RocketState &state = trajectory.states[si];
            using S = RocketState;
            // s.x > 0 =>  h(x) = x - k < 0 => x = - s.x, k = 0
            linear_constrains.emplace_back(si, S::i_position_y, 0, state.position().y(), min_position_y);
            // s.a > 0 => x = - s.a, k = 0
            linear_constrains.emplace_back(si, S::i_acceleration, 1, state.acceleration(), min_acceleration);
            linear_constrains.emplace_back(si, S::i_dt, 2, state.delta_time(), min_dt);
        }
    }

    std::vector<LinearConstrain1D> linear_constrains;

    // Rocket shoudn't hit ground
    double min_position_x = 0.;
    double min_position_y = 0.;

    // Acceleration is positive
    double min_acceleration = 0.;
    // dt is positive
    double min_dt = 1e-4;
};


struct Config
{
    Config()
    {
        weight_start << 1e4, 1e3, 1e3, 1e3, 1e3, 1e4, 1e4, 1e4;
        weight_end   << 1e4, 1e3, 1e3, 1e3, 1e3, 1e4, 1e4, 1e4;
    }

    // "sparse", "structural"
    std::string solve_type = "structural";

    uint32_t iterations = 20;

    double update_step_size = 0.5;

    Vector8d weight_start = Vector8d::Zero();
    
    Vector8d weight_end = Vector8d::Zero();

    int num_states = -1;
};

// RocketLandingProblem is not a name.
// It is a stuct contains data & operations that solver needs
struct RocketLandingProblem
{
    void update_problem()
    {
        assert(config.num_states >= 2);

        constrains.update_constrains(trajectory);
        residuals = compute_residaul(trajectory, 
            start_state, config.weight_start, end_state, config.weight_end, config.num_states);
    }

    Config config;

    // Variables
    Trajectory trajectory;

    // Residuals at current state. Need to be updated for new state.
    RocketLandingResiduals residuals;
    
    // Constrains
    Constrains constrains;

    // support data
    RocketState start_state;
    RocketState end_state;

};