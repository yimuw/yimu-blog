#pragma once

#include "solver_primal_dual.h"


// Block Diagonal
// e.g:
// | A1 A2          |
// | B1 B2 B3       |
// |    C1 C2 C3    |
// |       D1 D2 D3 |
// |          E1 E2 |
class BlockDiagonalSolver
{
public:
    // | A11 A12 | x1 = |b1|
    // | A21 A22 | x2 = |b2|
    struct EliminationBlock
    {
        EliminationBlock(const MatrixXd &iA11,
                         const MatrixXd &iA12,
                         const MatrixXd &iA11_inv,
                         const VectorXd &ib1)
            : A11(iA11), A12(iA12), A11_inv(iA11_inv), b1(ib1)
        {
        }

        MatrixXd A11;
        MatrixXd A12;
        MatrixXd A11_inv;
        VectorXd b1;
    };

// TODO: the const correctness of Eigen::block was messed up.
#define GET_MAT_STATE_BLOCK(lhs, num_state_rows, num_state_cols) \
lhs.block( (num_state_rows) * state_size_, (num_state_cols) * state_size_, state_size_, state_size_)

// TODO: the const correctness of Eigen::block was messed up.
#define GET_VECTOR_STATE_BLOCK(rhs, num_state_rows) \
rhs.block( (num_state_rows) * state_size_, 0, state_size_, 1)

    VectorXd solve(const NormalEqution &normal_equ, 
                    const int state_size,
                    const int num_states)
    {
        ScopeProfiler p("BlockDiagonalSolver_solve");

        state_size_ = state_size;
        num_states_ = num_states;

        assert(state_size_ * num_states_ == normal_equ.lhs.rows());
        assert(state_size_ * num_states_ == normal_equ.lhs.cols());
        assert(state_size_ * num_states_ == normal_equ.rhs.rows());

        if(false) PythonmatplotVisualizer().spy_matrix(normal_equ.lhs);

        const MatrixXd &lhs = normal_equ.lhs;
        const VectorXd &rhs = normal_equ.rhs;

        MatrixXd marginal_left = GET_MAT_STATE_BLOCK(lhs, 0, 0);
        VectorXd marginal_right = GET_VECTOR_STATE_BLOCK(rhs, 0);

        std::vector<EliminationBlock> elimination_chain;
        elimination_chain.reserve(num_states_ - 1);

        // eliminate by block
        for(int state_idx = 0; state_idx < num_states_ - 1; ++ state_idx)
        {
            const MatrixXd A11 = marginal_left;
            const MatrixXd A12 = GET_MAT_STATE_BLOCK(lhs, state_idx, state_idx + 1);
            const MatrixXd A21 = GET_MAT_STATE_BLOCK(lhs, state_idx + 1, state_idx);
            const MatrixXd A22 = GET_MAT_STATE_BLOCK(lhs, state_idx + 1, state_idx + 1);
            const VectorXd b1 = marginal_right;
            const VectorXd b2 = GET_VECTOR_STATE_BLOCK(rhs, state_idx + 1);
            const MatrixXd A11_inv = A11.inverse();

            marginal_left = (A22 - A21 * A11_inv * A12);
            marginal_right = b2 - A21 * A11_inv * b1;
            
            elimination_chain.emplace_back(A11, A12, A11_inv, b1);
        }

        assert(marginal_right.size() == state_size_);

        VectorXd x_result = VectorXd::Zero(state_size_ * num_states_);
        VectorXd x_last = marginal_left.inverse() * marginal_right;
        GET_VECTOR_STATE_BLOCK(x_result, num_states_ - 1) = x_last;

        // back substitition
        for(int x_idx = num_states_ - 2; x_idx >= 0; --x_idx)
        {
            EliminationBlock &eblock = elimination_chain.at(x_idx);
            x_last =  eblock.A11_inv * (eblock.b1 - eblock.A12 * x_last);
            GET_VECTOR_STATE_BLOCK(x_result, x_idx) = x_last;
        }

        return x_result;
    }
private:
    int state_size_;
    int num_states_;
};


class RocketLandingSolver_PrimalDualInteriorPointStructural 
    : public RocketLandingSolver_PrimalDualInteriorPoint
{
    virtual VectorXd solve_normal_eqution(NormalEqution &normal_equ) override
    {
        const int augmented_state_size = normal_equ.rhs.rows() / num_states_;
        VectorXd delta = BlockDiagonalSolver().solve(normal_equ, augmented_state_size, num_states_);
        
        constexpr bool TEST_SOLVER = false;
        if(TEST_SOLVER)
        {
            VectorXd delta_regression = SparseSolver::solve_normal_eqution(normal_equ);
            assert((delta_regression - delta).squaredNorm() < 1e-8 && "BlockDiagonalSolver failure");
        }
        delta = permute_delta(delta);
        return delta;
    }

    VectorXd permute_delta(const VectorXd &delta)
    {
        const int augmented_var_size = num_dual_variables_ + num_primal_variables_;
        VectorXd result = VectorXd::Zero(augmented_var_size);

        constexpr int state_size = RocketState::STATE_SIZE;
        // WARNING: hard coded!
        constexpr int constrains_size_per_size = 3;
        const int augmented_state_size = state_size + constrains_size_per_size;
        assert(augmented_state_size * num_states_ == augmented_var_size);

        for(int state_idx = 0; state_idx < num_states_; ++ state_idx)
        {
            const int state_start_idx = state_idx * state_size;
            const int dual_start_idx = num_states_ * state_size + constrains_size_per_size * state_idx;

            const int aug_state_start_idx = state_idx * augmented_state_size;
            
            // Dual residual
            result.block<state_size, 1>(state_start_idx, 0)
             = delta.block<state_size, 1>(aug_state_start_idx, 0);

            // Cent residual
            result.block<constrains_size_per_size, 1>(dual_start_idx, 0)
             = delta.block<constrains_size_per_size, 1>(aug_state_start_idx + state_size, 0);
        }

        return result;
    }

    // Solve for KKT: (Stationarity, Complementary slackness)
    virtual void construct_primal_dual_problem(const MatrixXd &cost_hessian,
                                                const VectorXd &cost_gradient,
                                                const VectorXd &dual_variables, 
                                                const Constrains &constrains,
                                                const double k_relax_complementary_slackness,
                                                NormalEqution &augmented_normal_equ) override
    {
        ScopeProfiler p("construct_primal_dual_problem_strutural");

        construct_primal_dual_problem_lhs(
            cost_hessian, dual_variables, constrains, augmented_normal_equ.lhs);

        augmented_normal_equ.rhs.setZero();
        augmented_normal_equ.rhs = construct_primal_dual_problem_rhs(
            cost_gradient, dual_variables, constrains, k_relax_complementary_slackness);

        if(false) PythonmatplotVisualizer().spy_matrix(augmented_normal_equ.lhs);
    }

        // Solve for KKT: (Stationarity, Complementary slackness)
    void construct_primal_dual_problem_lhs(const MatrixXd &cost_hessian,
                                           const VectorXd &dual_variables, 
                                           const Constrains &constrains,
                                           MatrixXd &augmented_normal_equ_lhs)
    {
        augmented_normal_equ_lhs.setZero();
        
        const int state_size = RocketState::STATE_SIZE;
        // WARNING: hard coded!
        const int constrains_size_per_size = 3;
        assert(static_cast<int>(constrains.linear_constrains.size()) == num_states_ * 3);
        const int augmented_state_size = state_size + constrains_size_per_size;

        // Very tricky
        for(int state_idx = 0; state_idx < num_states_; ++ state_idx)
        {
            const int state_start_idx = state_idx * state_size;
            const int aug_state_start_idx = state_idx * augmented_state_size;
            augmented_normal_equ_lhs.block<state_size, state_size>(aug_state_start_idx, aug_state_start_idx)
             = cost_hessian.block<state_size, state_size>(state_start_idx, state_start_idx);
            
            if(state_idx != num_states_ - 1)
            {
                augmented_normal_equ_lhs.block<state_size, state_size>(aug_state_start_idx + augmented_state_size, aug_state_start_idx)
                = cost_hessian.block<state_size, state_size>(state_start_idx + state_size, state_start_idx);

                augmented_normal_equ_lhs.block<state_size, state_size>(aug_state_start_idx, aug_state_start_idx + augmented_state_size)
                = cost_hessian.block<state_size, state_size>(state_start_idx, state_start_idx + state_size);
            }
        }

        // Not general, simplified for 1d linear case.
        for(size_t constrain_idx = 0; constrain_idx < constrains.linear_constrains.size(); ++constrain_idx)
        {
            const auto &constrain_linear = constrains.linear_constrains[constrain_idx];
            const int correspond_primal_index = constrain_linear.state_index * augmented_state_size
                + constrain_linear.type_index;
            const int dual_var_index = constrain_linear.state_index * augmented_state_size + state_size
                + constrain_linear.dual_index;
            // upper left
            // hessian for linear constrain is 0. Don't need to update it

            // upper right block
            augmented_normal_equ_lhs(correspond_primal_index, dual_var_index) = constrain_linear.jacobian();
            // lower left block
            augmented_normal_equ_lhs(dual_var_index, correspond_primal_index) 
                = - dual_variables[constrain_idx] * constrain_linear.jacobian();
            // lower right block
            augmented_normal_equ_lhs(dual_var_index, dual_var_index) = - constrain_linear.h();
        }
    }

    VectorXd construct_primal_dual_problem_rhs(const VectorXd &cost_gradient,
                                            const VectorXd &dual_variables, 
                                            const Constrains &constrains,
                                            const double k_relax_complementary_slackness)
    {
        const int num_variables = cost_gradient.rows();
        const int num_constrains = constrains.linear_constrains.size();
        const int augmented_var_size =  num_variables + num_constrains;
         
        constexpr int state_size = RocketState::STATE_SIZE;
        // WARNING: hard coded!
        const int constrains_size_per_size = 3;
        assert(static_cast<int>(constrains.linear_constrains.size()) == num_states_ * 3);
        const int augmented_state_size = state_size + constrains_size_per_size;

        VectorXd relaxed_kkt_rhs = VectorXd::Zero(augmented_var_size, 1);

        for(int state_idx = 0; state_idx < num_states_; ++ state_idx)
        {
            const int state_start_idx = state_idx * state_size;
            const int aug_state_start_idx = state_idx * augmented_state_size;
            relaxed_kkt_rhs.block<state_size, 1>(aug_state_start_idx, 0) 
                = -cost_gradient.block<state_size, 1>(state_start_idx, 0);
        }
        
        // Not general, simplified for 1d linear case.
        for(size_t constrain_idx = 0; constrain_idx < constrains.linear_constrains.size(); ++constrain_idx)
        {
            const auto &constrain_linear = constrains.linear_constrains[constrain_idx];
            const int correspond_primal_index = constrain_linear.state_index * augmented_state_size
                + constrain_linear.type_index;
            const int dual_var_index = constrain_linear.state_index * augmented_state_size + state_size
                + constrain_linear.dual_index;

            // Dual residual: KKT Stationarity
            relaxed_kkt_rhs(correspond_primal_index) -=  dual_variables[constrain_idx] * constrain_linear.jacobian();
            // Cent residual: KKT relaxed complementary slackness
            relaxed_kkt_rhs(dual_var_index) -= - dual_variables[constrain_idx] * constrain_linear.h() 
                - 1. / k_relax_complementary_slackness;
        }

        return relaxed_kkt_rhs;
    }
};