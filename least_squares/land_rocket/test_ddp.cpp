#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector2d = Eigen::Vector2d;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;
using TimeStamp = double;
using TimeDuration = double;

constexpr double dt = 1.;

struct RocketState {
    // Using a single dt is a better option.
    // But it make index handling a little bit more complex.
    enum StateIndex : int16_t {
        i_position_x = 0,
        i_position_y,
        i_velocity_x,
        i_velocity_y,
        STATE_SIZE
    };

    explicit RocketState(const double px,
        const double py,
        const double vel_x,
        const double vel_y)
    {
        variables << px, py, vel_x, vel_y;
    }

    RocketState() = default;

    static int variable_size()
    {
        return STATE_SIZE;
    }

    static int lcn_state_size()
    {
        return 2;
    }

    static int control_state_size()
    {
        // i_turning_rate,
        // i_acceleration,
        return 2;
    }

    Eigen::Map<Vector2d> lcn_state()
    {
        return Eigen::Map<Vector2d>(variables.data());
    }

    Eigen::Map<Vector2d> cntl_state()
    {
        return Eigen::Map<Vector2d>(variables.data() + 2);
    }

    Vector2d lcn_state() const
    {
        return variables.block<2, 1>(0, 0);
    }

    Vector2d cntl_state() const
    {
        return variables.block<2, 1>(2, 0);
    }

    Eigen::Map<Vector2d> position()
    {
        return Eigen::Map<Vector2d>(variables.data() + i_position_x);
    }

    Eigen::Map<Vector2d> velocity()
    {
        return Eigen::Map<Vector2d>(variables.data() + i_velocity_x);
    }

    Vector2d velocity() const
    {
        return { variables(i_velocity_x), variables(i_velocity_y) };
    }

    Vector4d variables = Vector4d::Zero();
};

struct RocketMotionModel {
    RocketState motion(const RocketState& state_in) const
    {
        auto state = state_in;
        // TODO: non-const getter
        RocketState next_state = state;

        next_state.velocity() = state.velocity();

        next_state.position()[0] = state.position()[0] + sin(state.velocity()[0]);
        next_state.position()[1] = state.position()[1] + state.velocity()[1];

        return next_state;
    }

    MatrixXd jacobian_wrt_state(const RocketState& state_in) const
    {
        using S = RocketState;
        const int residual_size = RocketState::STATE_SIZE;
        MatrixXd jacobi_wrt_s = MatrixXd::Identity(residual_size, residual_size);

        jacobi_wrt_s(S::i_position_x, S::i_velocity_x) = cos(state_in.velocity()[0]);
        jacobi_wrt_s(S::i_position_y, S::i_velocity_y) = 1.;

        return jacobi_wrt_s;
    }
};

#define PRINT_NAME_VAR(var) std::cout << #var << " :" << var << std::endl

using State = Vector2d;
using Control = Vector2d;

struct RocketLandingProblem {
    // Variables
    std::vector<RocketState> states = std::vector<RocketState>(11);

    // support data
    RocketState start_state = RocketState(0.1, 0.1, 0, 0);
    RocketState end_state = RocketState(2, 2, 0, 0);
};

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
        ddp_states.num_states = problem.states.size();
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
            ddp_states.states.at(i) = problem.states.at(i).lcn_state();
        }

        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            ddp_states.controls.at(i) = problem.states.at(i).cntl_state();
        }

        return ddp_states;
    }

    void update_output(const DDPStates& ddp_states, RocketLandingProblem& problem)
    {
        for (int i = 0; i < ddp_states.num_states; ++i) {
            problem.states.at(i).lcn_state() = ddp_states.states.at(i);
        }

        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            problem.states.at(i).cntl_state() = ddp_states.controls.at(i);
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

            std::cout << "idx forward iter: " << i << std::endl;
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
        const int state_size = 2;
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
        const int state_size = 2;
        const int control_size = 2;
        const int total_size = state_size + control_size;

        RocketMotionModel motion_model;
        const RocketState rstate = ddp_vars_to_rocket_state(cur_state, cur_control);

        const MatrixXd jacobi_wrt_augment_state = motion_model.jacobian_wrt_state(rstate);
        MatrixXd jacobian_marginal_cost = jacobi_wrt_augment_state.block(0, 0, state_size, total_size);

        const RocketState predict_state = motion_model.motion(rstate);

        std::cout << "rstate.var:       " << rstate.variables << std::endl;
        std::cout << "predict_state.var:" << predict_state.variables << std::endl;
        std::cout << "jacobian_marginal_cost: " << jacobian_marginal_cost << std::endl;

        const VectorXd residual = predict_state.lcn_state() - marginal_cost.mean;

        lhs = jacobian_marginal_cost.transpose() * marginal_cost.weight * jacobian_marginal_cost;
        rhs = -jacobian_marginal_cost.transpose() * marginal_cost.weight * residual;

        // current cost (just regularizations for this problem)
        const double time_regularization = 1e-3 / num_states_;
        lhs(RocketState::i_velocity_x, RocketState::i_velocity_x) += 2 * time_regularization;
        rhs(RocketState::i_velocity_x) += -2 * time_regularization * cur_control(RocketState::i_velocity_x - 2);

        lhs(RocketState::i_velocity_y, RocketState::i_velocity_y) += 2 * time_regularization;
        rhs(RocketState::i_velocity_y) += -2 * time_regularization * cur_control(RocketState::i_velocity_y - 2);
    }

    QuadraticCost target_cost(const Vector2d& target_state)
    {
        QuadraticCost qcost;
        qcost.mean = target_state;
        MatrixXd jacobi = MatrixXd::Identity(2, 2);

        MatrixXd weight = MatrixXd::Identity(2, 2);
        Vector2d weight_diag;
        //  dt, x, y, vel_x, vel_y, heading
        weight_diag << 1e0, 1e0;
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
        // for (; current_best_cost_ < shooting_cost && step > 1e-4; step *= 0.5) {
        for (int i = 0; i < ddp_states.num_states - 1; ++i) {
            const FeedbackLaw& feedback = ddp_states.feedbacks.at(i);
            Vector2d delta_state = new_states.at(i) - ddp_states.states.at(i);
            if (i == 0)
                assert(delta_state.sum() == 0);

            Vector2d delta_control = feedback.apply(delta_state);

            std::cout << "=== idex: " << i << std::endl;
            std::cout << "delta_state: " << delta_state.transpose() << std::endl;
            std::cout << "delta_control: " << delta_control.transpose() << std::endl;

            new_controls.at(i) = ddp_states.controls.at(i) + step * delta_control;

            const RocketState new_rstate = ddp_vars_to_rocket_state(new_states.at(i), new_controls.at(i));
            new_states.at(i + 1) = motion_model.motion(new_rstate).lcn_state();

            std::cout << "new_control: " << new_controls.at(i).transpose() << std::endl;
            std::cout << "cur_state:   " << new_states.at(i).transpose() << std::endl;
            std::cout << "new_state:   " << new_states.at(i + 1).transpose() << std::endl;
        }

        QuadraticCost marginal_cost = target_cost(ddp_states.target_state);
        double cur_cost = marginal_cost.eval(new_states.back());
        shooting_cost = std::min(shooting_cost, cur_cost);
        // }

        current_best_cost_ = std::min(current_best_cost_, shooting_cost);

        std::cout << "ddp backtracking step:" << step << std::endl;
        std::cout << "current_best_cost_:" << current_best_cost_ << std::endl;
        std::cout << "cur_cost:" << cur_cost << std::endl;

        ddp_states.states = new_states;
        ddp_states.controls = new_controls;
    }

    // current minmum target cost. Should use total cost.
    double current_best_cost_ = std::numeric_limits<double>::max();

    double num_states_ = -1;
};

// yimu@yimu-mate:~/Desktop/blog$ /usr/bin/python3 /home/yimu/Desktop/blog/yimu-blog/least_squares/ddp/ddp_optimization.py
// initial_state: [0.1 0.1]
// target_state: [2. 2.]
// num_states: 11
// final_state_end_cost: [[0.22966875]]
// final_state_end_cost: [[0.00102592]]
// final_state_end_cost: [[0.00737957]]
// final_state_end_cost: [[0.00287627]]
// final_state_end_cost: [[0.00056493]]
// final_state_end_cost: [[6.04254466e-05]]
// final_state_end_cost: [[1.60630207e-06]]
// final_state_end_cost: [[8.4323494e-07]]
// final_state_end_cost: [[1.20808104e-06]]
// final_state_end_cost: [[6.61618857e-07]]
// ----------------------------------
// new_controls:
//  [array([0.19102753, 0.18981443]), array([0.19110838, 0.18991752]), array([0.19117968, 0.19001093]), array([0.19123781, 0.19009055]), array([0.19127743, 0.19015008]), array([0.19129027, 0.19017957]), array([0.19126273, 0.19016206]), array([0.19117054, 0.19006576]), array([0.19096421, 0.18982076]), array([0.19051605, 0.18921421])]
// new_states:
//  [array([0.1, 0.1]), array([0.28986784, 0.28981443]), array([0.47981505, 0.47973195]), array([0.66983226, 0.66974288]), array([0.85990655, 0.85983343]), array([1.05001973, 1.04998351]), array([1.24014552, 1.24016308]), array([1.43024427, 1.43032515]), array([1.62025251, 1.6203909 ]), array([1.81005818, 1.81021167]), array([1.9994238 , 1.99942588])]

// ./test_ddp | grep cur_cost
// cur_cost:0.229777
// cur_cost:0.00101972
// cur_cost:0.00736767
// cur_cost:0.00287048
// cur_cost:0.000562644
// cur_cost:5.97038e-05
// cur_cost:1.5031e-06
// cur_cost:9.30061e-07
// cur_cost:1.31691e-06
// cur_cost:7.43505e-07
//
// new_control: 0.190882 0.189625
// cur_state:   0.1 0.1
// --
// new_control: 0.191034 0.189811
// cur_state:   0.289725 0.289625
// --
// new_control: 0.191173 0.189985
// cur_state:   0.479599 0.479436
// --
// new_control: 0.191293 0.190141
// cur_state:    0.66961 0.669421
// --
// new_control: 0.191385 0.190269
// cur_state:   0.859738 0.859562
// --
// new_control: 0.191435 0.190352
// cur_state:   1.04996 1.04983
// --
// new_control: 0.191417 0.190362
// cur_state:   1.24023 1.24018
// --
// new_control:  0.19128 0.190237
// cur_state:   1.43048 1.43055
// --
// new_control:  0.19091 0.189834
// cur_state:   1.62059 1.62078
// --
// new_control: 0.189926 0.188637
// cur_state:   1.81034 1.81062
// --
// new_control: 0.191024 0.189811
// cur_state:   0.1 0.1
// --
// new_control: 0.191105 0.189914
// cur_state:   0.289864 0.289811
// --
// new_control: 0.191176 0.190007
// cur_state:   0.479808 0.479725
// --
// new_control: 0.191234 0.190087
// cur_state:   0.669822 0.669733
// --
// new_control: 0.191274 0.190147
// cur_state:   0.859892  0.85982
// --
// new_control: 0.191287 0.190176
// cur_state:      1.05 1.04997
// --
// new_control: 0.191259 0.190159
// cur_state:   1.24012 1.24014
// --
// new_control: 0.191167 0.190062
// cur_state:   1.43022  1.4303
// --
// new_control: 0.190961 0.189817
// cur_state:   1.62022 1.62036
// --
// new_control: 0.190513 0.189211
// cur_state:   1.81003 1.81018

int main(int argc, char* argv[])
{
    RocketLandingProblem p;

    DifferentialDynamicProgramming solver;
    solver.solve(p);

    return 1;
}