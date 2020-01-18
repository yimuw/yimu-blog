/*
Tests to develop solvers
 */

#include <iostream>

// TODO: expicit includes
#include "solver.h"
#include "rocket_landing_planner.h"
#include "least_square_problem.h"
#include "utils.h"


void test_matrix_almost_equation(const MatrixXd &m1, const MatrixXd &m2)
{
    constexpr double EPSILON = 1e-4;

    assert(m1.rows() == m2.rows() && "m1.rows == m2.row failed");
    assert(m1.cols() == m2.cols() && "m1.cols == m2.cols failed");

    for(int r = 0; r < m1.rows(); ++r)
    {
        for(int c = 0; c < m1.cols(); ++c)
        {
            double diff = m1(r,c) - m2(r, c);
            assert(std::abs(diff) < EPSILON && "matrix_almost_equation failed");
        }
    }

    std::cout << "test_matrix_almost_equation pass" << std::endl;
}

// Helper class to test private functions
class Test_RocketLandingPlanner : public RocketLandingPlanner
{
public:
    // use base constructor
    using RocketLandingPlanner::RocketLandingPlanner;
    
    // test protected functions
    RocketLandingProblem TEST_get_problem()
    {
        return problem_;
    }

    Config TEST_get_config()
    {
        return config_;
    }
};

RocketLandingResiduals generate_data()
{
    RocketState start_state(1., 0., 0., 0., 0., 0., 0., 0.);
    RocketState end_state(1., 100., 100., 0., 0., 0., 0., 0.);
    int steps = 20;

    Test_RocketLandingPlanner rocket_landing(start_state, end_state, steps);
    RocketLandingResiduals r = rocket_landing.TEST_get_problem().residuals;
    
    return r;
}

template<typename SOLVER_TYPE>
VectorXd solve_by(const RocketLandingResiduals &r)
{
    SOLVER_TYPE solver;
    VectorXd delta = solver.solver_rocket_landing_least_squares(r);
    return delta;
}

void test_solvers()
{
    const RocketLandingResiduals r = generate_data();

    // ground truth
    VectorXd delta_dense = solve_by<DenseSolver>(r);

    VectorXd delta_dense_in_place = solve_by<DenseSolverUpdateInPlace>(r);
    test_matrix_almost_equation(delta_dense_in_place, delta_dense);

    VectorXd sparse_delta = solve_by<SparseSolver>(r);
    test_matrix_almost_equation(sparse_delta, delta_dense);
}


struct TEST_RocketLandingSolver_PrimalDualInteriorPoint : public RocketLandingSolver_PrimalDualInteriorPoint
{
    void cost_gradient_checking(const Config &config,
                                RocketLandingProblem &problem)
    {
        config_ = config;

        num_states_ = problem.trajectory.states.size();
        num_primal_variables_ = problem.trajectory.states.size() * RocketState::STATE_SIZE;
        num_dual_variables_ = problem.constrains.linear_constrains.size();

        problem.update_problem();

        print_variables(problem.trajectory);
        const auto trajectory_cur = problem.trajectory;

        VectorXd cost_grad_nu(num_primal_variables_);

        auto compute_cost = [&](const Trajectory &t)
        {
            auto residuals = compute_residaul(t, 
                problem.start_state, config.weight_start, problem.end_state, 
                config.weight_end, config.num_states);
            return residuals.total_cost();
        };

        for(size_t state_i = 0; state_i < trajectory_cur.states.size(); ++state_i)
        {
            for(size_t var_i = 0; var_i < RocketState::STATE_SIZE; ++var_i)
            {
                constexpr double EPSILON = 1e-8;
                auto t_plus = trajectory_cur;
                t_plus.states.at(state_i).variables(var_i) += EPSILON;
                auto t_minus = trajectory_cur;
                t_minus.states.at(state_i).variables(var_i) -= EPSILON;

                const double grad = (compute_cost(t_plus) - compute_cost(t_minus)) / (2 * EPSILON);
                cost_grad_nu(state_i * RocketState::STATE_SIZE + var_i) = grad;
            }
        }

        NormalEqution normal_equ = residual_function_to_normal_equation(problem.residuals);
        VectorXd cost_grad_analytic = compute_cost_gradient(normal_equ, problem.trajectory, problem.residuals);
        PRINT_NAME_VAR(problem.residuals.total_cost());
        PRINT_NAME_VAR(cost_grad_analytic.transpose());
        PRINT_NAME_VAR(cost_grad_nu.transpose());
    }
};

void cost_gradient_checking()
{
    RocketState start_state(1., 0.1, 0.2, 0.3, 0.3,  0.4, 0.5, 0.6);
    RocketState end_state(1., 1., 0.5, 0.1, 0., 0., 0., 0.);

    int steps = 2;
    Test_RocketLandingPlanner tester_landing(start_state, end_state, steps);

    TEST_RocketLandingSolver_PrimalDualInteriorPoint tester_solver;
    auto problem_for_test = tester_landing.TEST_get_problem();
    tester_solver.cost_gradient_checking(tester_landing.TEST_get_config(), problem_for_test);
}

int main(int argc, char *argv[])
{
    std::cout << "test_solvers =========================================" << std::endl;
    test_solvers();

    std::cout << "cost_gradient_checking =========================================" << std::endl;
    cost_gradient_checking();

    return 0;
}