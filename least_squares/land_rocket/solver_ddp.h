#pragma once

#include <eigen3/Eigen/Dense>

#include "solver_naive.h"

using State = Vector6d;
using Control = Vector2d;

struct QuadraticCost {
    QuadraticCost() = default;

    QuadraticCost(const VectorXd& meanIn, const MatrixXd& weightIn)
        : mean(meanIn)
        , weight(weightIn)
    {
    }

    double eval(const VectorXd& x) const
    {
        return (x - mean).transpose() * weight * (x - mean);
    }

    VectorXd mean;
    MatrixXd weight;
};

// du = K1 * dx + k2
struct FeedbackLaw {
    FeedbackLaw() = default;

    FeedbackLaw(MatrixXd K1, VectorXd k2)
        : K1(K1)
        , k2(k2)
    {
    }

    VectorXd apply(const VectorXd& x) const
    {
        return K1 * x + k2;
    }

    MatrixXd K1;
    VectorXd k2;
};

// The definition of states and controls is differient from previous section.
struct DDPStates {
    State last_state()
    {
        return states.at(num_states - 1);
    }

    int num_states = 0;
    std::vector<Control> controls;
    std::vector<State> states;
    std::vector<FeedbackLaw> feedbacks;

    State init_state;
    State target_state;
};

inline RocketState ddp_vars_to_rocket_state(const State& x, const Control& u)
{
    RocketState rstate;
    rstate.cntl_state() = u;
    rstate.lcn_state() = x;
    return rstate;
}

class DifferentialDynamicProgramming {
public:
    void solve(RocketLandingProblem& problem)
    {
        DDPStates ddp_states = initialize_ddp_state(problem);
        for (int iteration = 0; iteration < 10; ++iteration) {
            forward_pass(ddp_states);
            backward_pass(ddp_states);
            apply_control_law(ddp_states);
        }

        update_output(ddp_states, problem);
    }

private:
    DDPStates initialize_ddp_state(const RocketLandingProblem& problem)
    {
        DDPStates ddp_states;
        ddp_states.num_states = problem.trajectory.states.size();
        num_states_ = ddp_states.num_states;
        ddp_states.init_state = problem.start_state.lcn_state();
        ddp_states.target_state = problem.end_state.lcn_state();

        PRINT_NAME_VAR(ddp_states.num_states);
        PRINT_NAME_VAR(ddp_states.init_state);
        PRINT_NAME_VAR(ddp_states.target_state);

        ddp_states.states.resize(ddp_states.num_states);
        ddp_states.controls.resize(ddp_states.num_states - 1);
        ddp_states.feedbacks.resize(ddp_states.num_states - 1);

        for (int i = 0; i < ddp_states.num_states; ++i) {
            ddp_states.states.at(i) = problem.trajectory.states.at(i).lcn_state();
        }

        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            ddp_states.controls.at(i) = problem.trajectory.states.at(i).cntl_state();
        }

        return ddp_states;
    }

    void update_output(const DDPStates& ddp_states, RocketLandingProblem& problem)
    {
        for (int i = 0; i < ddp_states.num_states; ++i) {
            problem.trajectory.states.at(i).lcn_state() = ddp_states.states.at(i);
        }

        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            problem.trajectory.states.at(i).cntl_state() = ddp_states.controls.at(i);
        }
    }

    void forward_pass(DDPStates& ddp_states)
    {
        ddp_states.states.resize(ddp_states.num_states);
        ddp_states.controls.resize(ddp_states.num_states - 1);

        ddp_states.states[0] = ddp_states.init_state;

        RocketMotionModel motion_model;
        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            RocketState current_full_state = ddp_vars_to_rocket_state(
                ddp_states.states.at(i), ddp_states.controls.at(i));

            RocketState next_full_state = motion_model.motion(current_full_state);
            ddp_states.states.at(i + 1) = next_full_state.lcn_state();

            std::cout << "forward iter: " << i << std::endl;
            std::cout << "control: " << ddp_states.controls.at(i).transpose() << std::endl;
            std::cout << "state: " << ddp_states.states.at(i).transpose() << std::endl;
            std::cout << "next state: " << ddp_states.states.at(i + 1).transpose() << std::endl;
        }
    }

    void backward_pass(DDPStates& ddp_states)
    {
        QuadraticCost marginal_cost = target_cost(ddp_states.target_state);

        current_best_cost_ = std::min(current_best_cost_, marginal_cost.eval(ddp_states.last_state()));
        std::cout << "target cost :" << marginal_cost.eval(ddp_states.last_state()) << std::endl;

        for (int i = ddp_states.num_states - 2; i >= 0; --i) {
            const State cur_state = ddp_states.states.at(i);
            const Control cur_control = ddp_states.controls.at(i);

            solve_backward_subproblem(cur_state, cur_control, marginal_cost, ddp_states.feedbacks[i]);
        }
    }

    void solve_backward_subproblem(const State& cur_state,
        const Control& cur_control,
        QuadraticCost& marginal_cost,
        FeedbackLaw& feedback_law)
    {
        const int state_size = 6;
        const int control_size = 2;

        MatrixXd lhs;
        VectorXd rhs;
        compute_backward_subproblem_normal_equation(marginal_cost, cur_state, cur_control, lhs, rhs);

        MatrixXd A1 = lhs.block(0, 0, state_size, state_size);
        MatrixXd A2 = lhs.block(0, state_size, state_size, control_size);
        MatrixXd A3 = lhs.block(state_size, 0, control_size, state_size);
        MatrixXd A4 = lhs.block(state_size, state_size, control_size, control_size);

        VectorXd b1 = rhs.block(0, 0, state_size, 1);
        VectorXd b2 = rhs.block(state_size, 0, control_size, 1);

        MatrixXd A4_inv = A4.inverse();

        // eliminate du
        MatrixXd lhs_xi = A1 - A2 * A4_inv * A3;
        VectorXd rhs_xi = b1 - A2 * A4_inv * b2;

        // solve PSD system by Cholesky
        VectorXd xi_star = lhs_xi.llt().solve(rhs_xi);

        marginal_cost = QuadraticCost(cur_state + xi_star, lhs_xi);
        // std::cout << "marginal_cost \n mean:" << marginal_cost.mean.transpose()
        //     << "\n weight:" << marginal_cost.weight << std::endl;
        feedback_law = FeedbackLaw(-A4_inv * A3, A4_inv * b2);
    }

    void compute_backward_subproblem_normal_equation(const QuadraticCost& marginal_cost,
        const State& cur_state,
        const Control& cur_control,
        MatrixXd& lhs,
        VectorXd& rhs)
    {
        const int state_size = 6;
        const int control_size = 2;
        const int total_size = state_size + control_size;

        RocketMotionModel motion_model;
        const RocketState rstate = ddp_vars_to_rocket_state(cur_state, cur_control);

        const MatrixXd jacobi_wrt_augment_state = motion_model.jacobian_wrt_state(rstate);
        MatrixXd jacobian_marginal_cost = jacobi_wrt_augment_state.block(0, 0, state_size, total_size);

        const VectorXd residual = motion_model.motion(rstate).lcn_state() - marginal_cost.mean;

        lhs = jacobian_marginal_cost.transpose() * marginal_cost.weight * jacobian_marginal_cost;
        rhs = -jacobian_marginal_cost.transpose() * marginal_cost.weight * residual;

        // current cost (just regularizations for this problem)
        const double time_regularization = 1e-3 / num_states_;
        lhs(RocketState::i_dt, RocketState::i_dt) += 2 * time_regularization;
        rhs(RocketState::i_dt) += -2 * time_regularization * cur_state(RocketState::i_dt);

        const double acceleration_regularization = 1e-1 / num_states_;
        lhs(RocketState::i_acceleration, RocketState::i_acceleration) += 2 * acceleration_regularization;
        rhs(RocketState::i_acceleration) += -2 * acceleration_regularization * cur_control(RocketState::i_acceleration - state_size);

        const double turning_rate_regularization = 1e0 / num_states_;
        lhs(RocketState::i_turning_rate, RocketState::i_turning_rate) += 2 * turning_rate_regularization;
        rhs(RocketState::i_turning_rate) += -2 * turning_rate_regularization * cur_control(RocketState::i_turning_rate - state_size);
    }

    QuadraticCost target_cost(const Vector6d& target_state)
    {
        QuadraticCost qcost;
        qcost.mean = target_state;
        MatrixXd jacobi = MatrixXd::Identity(6, 6);

        MatrixXd weight = MatrixXd::Identity(6, 6);
        Vector6d weight_diag;
        //  dt, x, y, vel_x, vel_y, heading
        weight_diag << 1e2, 1e1, 1e1, 1e1, 1e1, 1e0;
        weight.diagonal() = weight_diag;

        qcost.weight = jacobi.transpose() * weight * jacobi;

        return qcost;
    }

    void apply_control_law(DDPStates& ddp_states)
    {
        std::vector<Control> new_controls = ddp_states.controls;
        std::vector<State> new_states = ddp_states.states;
        RocketMotionModel motion_model;

        double shooting_cost = std::numeric_limits<double>::max();

        PRINT_NAME_VAR(shooting_cost);
        PRINT_NAME_VAR(current_best_cost_);

        // DDP is HARD to converge!
        double step = 0.5;
        for (; current_best_cost_ < shooting_cost && step > 1e-4; step *= 0.5) {
            for (int i = 0; i < ddp_states.num_states - 1; ++i) {
                const FeedbackLaw& feedback = ddp_states.feedbacks.at(i);
                Vector6d delta_state = new_states.at(i) - ddp_states.states.at(i);
                Vector2d delta_control = feedback.apply(delta_state);

                new_controls.at(i) = ddp_states.controls.at(i) + step * delta_control;

                const RocketState new_rstate = ddp_vars_to_rocket_state(new_states.at(i), new_controls.at(i));
                new_states.at(i + 1) = motion_model.motion(new_rstate).lcn_state();
            }

            QuadraticCost marginal_cost = target_cost(ddp_states.target_state);
            shooting_cost = std::min(shooting_cost, marginal_cost.eval(new_states.back()));
        }

        current_best_cost_ = std::min(current_best_cost_, shooting_cost);

        std::cout << "ddp backtracking step:" << step << std::endl;
        std::cout << "current_best_cost_:" << current_best_cost_ << std::endl;

        ddp_states.states = new_states;
        ddp_states.controls = new_controls;
    }

    // current minmum target cost. Should use total cost.
    double current_best_cost_ = std::numeric_limits<double>::max();

    double num_states_ = -1;

    RocketLandingProblem problem_;
};