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

    MatrixXd jocobian_wrt_state(const RocketState &state_in) const
    {
        using S = RocketState;
        const int residual_size = RocketState::STATE_SIZE;
        MatrixXd jacobi_wrt_s = MatrixXd::Identity(residual_size, residual_size);
        
        const double h1 = state_in.heading();
        const double dt1 = state_in.delta_time();

        jacobi_wrt_s(S::i_position_x, S::i_heading) = - dt1 * std::sin(h1) * state_in.velocity();
        jacobi_wrt_s(S::i_position_x, S::i_velocity) = dt1 * std::cos(h1);
        jacobi_wrt_s(S::i_position_x, S::i_dt) = std::cos(h1) * state_in.velocity();

        jacobi_wrt_s(S::i_position_y, S::i_heading) = dt1 * std::cos(h1) * state_in.velocity();
        jacobi_wrt_s(S::i_position_y, S::i_velocity) = dt1 * std::sin(h1);
        jacobi_wrt_s(S::i_position_y, S::i_dt) = std::sin(h1) * state_in.velocity();

        jacobi_wrt_s(S::i_velocity, S::i_acceleration) = dt1;
        jacobi_wrt_s(S::i_velocity, S::i_dt) = state_in.acceleration();

        jacobi_wrt_s(S::i_heading, S::i_turning_rate) = dt1;
        jacobi_wrt_s(S::i_heading, S::i_dt) = state_in.turning_rate();
        
        return jacobi_wrt_s;
    }

    // f(x + dx) = J(x) * dt + x
    // technically, It is not jacobian checking. No reliabale...
    void jacobian_checking_simple(const RocketState &state) const
    {
        constexpr double EPSILON = 1e-1;
        Vector7d delta = EPSILON * Vector7d::Ones();
        MatrixXd jacobi = jocobian_wrt_state(state);

        RocketState state_pred;
        state_pred.variables = state.variables + delta;
        const Vector7d state_gt = motion(state_pred).variables;
        const Vector7d state_linearized = state.variables + jacobi * delta;

        std::cout << "state_gt - state_linearized: " << state_gt - state_linearized << std::endl;
        std::cout << "EPSILON: " << EPSILON << std::endl;
    }
};


// NOTE: indexing is not supported. We need to carefully handle the index to variable mapping
struct Residual
{
    virtual ~Residual(){};

    virtual MatrixXd jacobian() const = 0;

    virtual MatrixXd weight() const = 0;

    virtual bool is_diagnal_weight() const = 0;

    virtual VectorXd residual() const = 0;

    virtual int residual_size() const = 0;

    virtual int variable_size() const = 0;

    // assume variable block
    virtual int variable_start_index() const = 0;

    double cost() const
    {
        const VectorXd r = residual();
        return r.transpose() * weight() * r;
    }
};


struct MotionResidual : public Residual
{
    // support container
    MotionResidual() = default;

    MotionResidual(const RocketState &s1, 
                   const int s1_idx, 
                   const RocketState &s2,
                   const int s2_idx,
                   const double w_scale)
        : weight_scale(w_scale), state1_index(s1_idx), state2_index(s2_idx), 
          state1(s1), state2(s2) 
    {}

    MatrixXd jacobian() const override
    {
        const int residual_size = state1.variable_size();
        const int variable_size = 2 * state1.variable_size();

        MatrixXd jacobi = MatrixXd::Zero(residual_size, variable_size);

        const MatrixXd jacobi_wrt_s1 = RocketMotionModel().jocobian_wrt_state(state1);
        const MatrixXd jacobi_wrt_s2 = - MatrixXd::Identity(residual_size, residual_size);

        jacobi.block(0,0,residual_size, residual_size) = jacobi_wrt_s1;
        jacobi.block(0,residual_size,residual_size, residual_size) = jacobi_wrt_s2;

        return jacobi;
    }

    // r(x1, x2) = m(x1) - x2
    VectorXd residual() const override
    {
        // RocketMotionModel().jacobian_checking_simple(state1);

        RocketState state2_pred = RocketMotionModel().motion(state1);
        Vector7d r = state2_pred.variables - state2.variables;

        return r;
    }

    int residual_size() const override
    {
        return RocketState::STATE_SIZE;
    }

    int variable_size() const override
    {
        return 2 * RocketState::STATE_SIZE;
    }

    int variable_start_index() const override
    {
        assert(state1_index + 1 == state2_index);
        return state1_index * RocketState::STATE_SIZE;
    }

    MatrixXd weight() const override
    {
        const int residual_size = state1.variable_size();
        Vector7d weight_diag;

        //  dt, x, y, vel, heading, turn_rate, accl
        weight_diag << 1e2, 1e3, 1e3, 1e4, 1e4, 1e1, 1e1;
        // weight_diag = Vector7d::Ones();

        Matrix7d weight = MatrixXd::Identity(residual_size, residual_size);
        weight.diagonal() << weight_scale * weight_diag;

        return weight;
    }

    bool is_diagnal_weight() const override
    {
        return true;
    }

    double weight_scale = 1.;

    int state1_index = -1;
    int state2_index = -1;
    RocketState state1;
    RocketState state2;
};

struct PriorResidual : public Residual
{
    PriorResidual() = default;

    PriorResidual(const RocketState &s, 
                  const int s_idx, 
                  const RocketState &p,
                  const Vector7d &w)
        : state_index(s_idx), state(s), prior(p), weight_vec(w)
    {}

    MatrixXd jacobian() const override
    {
        const int residual_size = state.variable_size();

        MatrixXd jacobi = MatrixXd::Identity(residual_size, residual_size);
        return jacobi;
    }

    // r(x1, x2) = m(x1) - x2
    VectorXd residual() const override
    {
        Vector7d r = state.variables - prior.variables;
        return r;
    }

    MatrixXd weight() const override
    {
        const int state_size = RocketState::STATE_SIZE;
        Matrix7d weight_mat = MatrixXd::Zero(state_size, state_size);
        assert(weight_vec.rows() == state_size);
        weight_mat.diagonal() = weight_vec;

        return weight_mat;
    }

    bool is_diagnal_weight() const override
    {
        return true;
    }

    int residual_size() const override
    {
        return RocketState::STATE_SIZE;
    }

    int variable_size() const override
    {
        return RocketState::STATE_SIZE;
    }

    int variable_start_index() const override
    {
        return state_index * RocketState::STATE_SIZE;
    }

    int state_index = -1;
    RocketState state;

    RocketState prior;
    Vector7d weight_vec = Vector7d::Ones();
};


// NOTE: Regularization can be implemented by adding diagnal terms to A^T * A.
struct RegularizationResidual
{
};
