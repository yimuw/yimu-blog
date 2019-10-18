#pragma once

#include "types.h"


struct RocketMotionModel
{
    RocketState motion(const RocketState &state_in) const
    {
        auto state = state_in;
        // TODO: non-const getter
        RocketState next_state = state;
        const double dt = state.delta_time();
        next_state.velocity() = state.velocity() + dt * state.acceleration();
        next_state.heading() = state.heading() + dt * state.turning_rate();
        const double theta = state.heading();
        next_state.position() = state.position() + dt * Vector2d(std::cos(theta) * state.velocity(), 
                                                                 std::sin(theta) * state.velocity()); 
        return next_state;
    }
};


// NOTE: indexing is not supported. We need to carefully handle the index to variable mapping
struct Residual
{
    virtual ~Residual(){};

    virtual MatrixXd jacobian() const = 0;

    virtual MatrixXd weight() const = 0;

    virtual VectorXd residual() const = 0;
};


struct MotionResidual : public Residual
{
    // support container
    MotionResidual() = default;

    MotionResidual(const RocketState &s1, 
                   const int s1_idx, 
                   const RocketState &s2,
                   const int s2_idx)
        : state1(s1), state1_index(s1_idx), state2(s2), state2_index(s2_idx)
    {}

    MatrixXd jacobian() const override
    {
        const int residual_size = state1.variable_size();
        const int variable_size = 2 * state1.variable_size();

        MatrixXd jacobi = MatrixXd::Zero(residual_size, variable_size);

        using S = RocketState;
        MatrixXd jacobi_wrt_s1 = MatrixXd::Identity(residual_size, residual_size);
        {
            const double h1 = state1.heading();
            const double dt1 = state1.delta_time();
            jacobi_wrt_s1(S::i_position_x, S::i_heading) = - dt1 * std::sin(h1) * state1.velocity();
            jacobi_wrt_s1(S::i_position_x, S::i_velocity) = dt1 * std::cos(h1);
            jacobi_wrt_s1(S::i_position_y, S::i_heading) = dt1 * std::cos(h1) * state1.velocity();
            jacobi_wrt_s1(S::i_position_y, S::i_velocity) = dt1 * std::sin(h1);
            jacobi_wrt_s1(S::i_velocity, S::i_acceleration) = dt1;
            jacobi_wrt_s1(S::i_heading, S::i_turning_rate) = dt1;
        }

        MatrixXd jacobi_wrt_s2 = - MatrixXd::Identity(residual_size, residual_size);

        jacobi.block(0,0,residual_size, residual_size) = jacobi_wrt_s1;
        jacobi.block(residual_size,residual_size,residual_size, residual_size) = jacobi_wrt_s2;

        return jacobi;
    }

    // r(x1, x2) = m(x1) - x2
    VectorXd residual() const override
    {
        RocketState state2_pred = RocketMotionModel().motion(state1);
        VectorXd r = state2_pred.variables - state2.variables;
    }

    int residual_size() const
    {
        return RocketState::STATE_SIZE;
    }

    int variable_size() const
    {
        return 2 * RocketState::STATE_SIZE;
    }

    MatrixXd weight() const override
    {
        const int residual_size = state1.variable_size();
        MatrixXd weight = MatrixXd::Identity(residual_size, residual_size);
        using S = RocketState;
        weight(S::i_heading, S::i_heading) = 1e-1;
        weight(S::i_turning_rate, S::i_turning_rate) = 1e-1;

        return weight;
    }

    int state1_index = -1;
    int state2_index = -1;
    RocketState state1;
    RocketState state2;
};

struct PriorResidual : public Residual
{
    PriorResidual() = default;

    PriorResidual(const RocketState &s, const int s_idx, const RocketState &p)
        : state(s), state_index(s_idx), prior(p)
    {}

    MatrixXd jacobian() const override
    {
        const int residual_size = state.variable_size();
        const int variable_size = state.variable_size();

        MatrixXd jacobi = MatrixXd::Identity(residual_size, residual_size);
        return jacobi;
    }

    // r(x1, x2) = m(x1) - x2
    VectorXd residual() const override
    {
        VectorXd r = state.variables - prior.variables;
    }

    MatrixXd weight() const override
    {
        const int residual_size = state.variable_size();
        Vector7d weight_diag;
        // enum StateIndex
        // {
        //     i_dt = 0,
        //     i_position_x,
        //     i_position_y,
        //     i_velocity,
        //     i_heading,
        //     i_turning_rate,
        //     i_acceleration,
        //     STATE_SIZE
        // };
        weight_diag << 1, 100, 100, 100, 10, 10, 100;
        MatrixXd weight = Eigen::Diagonal(weight_diag);

        return weight;
    }

    int residual_size() const
    {
        return RocketState::STATE_SIZE;
    }

    int variable_size() const
    {
        return RocketState::STATE_SIZE;
    }

    int state_index = -1;
    RocketState state;

    RocketState prior;
};


// NOTE: Regularization can be implemented by adding diagnal terms to A^T * A.
struct RegularizationResidual
{
};


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
    
    std::vector<MotionResidual> motion_residuals;
    PriorResidual start_state_prior;
    PriorResidual end_state_prior;

    int num_rocket_states = -1;
};