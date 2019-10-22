/*
Tests to develop solvers
 */

#include <iostream>

// TODO: expicit includes
#include "solver.h"
#include "rocket_landing_planner.h"
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
    RocketLandingResiduals TEST_compute_residaul()
    {
        return compute_residaul(trajectory_, 
                start_state_, config_.weight_start, end_state_, config_.weight_end, num_states_);
    }
};

RocketLandingResiduals generate_data()
{
    RocketState start_state(1., 0., 0., 0., 0., 0., 0.);
    RocketState end_state(1., 100., 100., 0., 0., 0., 0.);
    int steps = 20;

    Test_RocketLandingPlanner rocket_landing(start_state, end_state, steps);
    RocketLandingResiduals r = rocket_landing.TEST_compute_residaul();
    
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

// No a good test
void test_residual()
{
    RocketState s1(1, 1, 1, 1, 1, 1, 1);
    RocketState s2(2, 2, 2, 2, 2, 2, 2);

    const Vector7d weight = 3. * Vector7d::Ones();

    Trajectory trajectory;

    trajectory.states = {s1 , s2};
    s1.delta_time() += 1.23;

    const RocketLandingResiduals residuals = compute_residaul(trajectory, 
                s1, weight, s2, weight, 2);
    
    const double cost_pred = residuals.total_cost();

    auto s2_pred = RocketMotionModel().motion(trajectory.states.at(0));
    const Vector7d diff = s2_pred.variables - trajectory.states.at(1).variables;

    const double t_reg = residuals.time_regularization;
    const double v_reg = residuals.velocity_regularization;
    const double a_reg = residuals.acceleration_regularization;
    const double r_reg = residuals.turning_rate_regularization;

    auto reg_compute = [&](const RocketState &s)
    {
        return t_reg * s.delta_time() * s.delta_time()
            +  v_reg * s.velocity() * s.velocity()
            +  a_reg * s.acceleration() * s.acceleration()
            +  r_reg * s.turning_rate() * s.turning_rate();
    };

    MatrixXd W = residuals.motion_residuals[0].weight();
    const double cost_gt = diff.transpose() * W * diff + 1.23 * 1.23 * 3.
        + reg_compute(trajectory.states.at(0)) + reg_compute(trajectory.states.at(1));
    PRINT_NAME_VAR(cost_gt);
    PRINT_NAME_VAR(cost_pred);
    assert(std::abs(cost_gt - cost_pred) < 1e-8 && "cost checking failed");
    std::cout << "test_residual passed" << std::endl;
}

int main(int argc, char *argv[])
{
    test_solvers();

    test_residual();

    return 0;
}