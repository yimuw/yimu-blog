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
        return this->compute_residaul();
    }
};

RocketLandingResiduals generate_data()
{
    //  dt, x, y, vel, heading, turn_rate, accl
    RocketState start_state(1., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
    RocketState end_state(1., 10., 10., 0., 0., 0., 0.);
    int steps = 5;

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
    RocketLandingResiduals r = generate_data();

    VectorXd delta_dense = solve_by<DenseSolver>(r);
    PRINT_NAME_VAR(delta_dense.transpose());

    VectorXd delta_dense_in_place = solve_by<DenseSolverUpdateInPlace>(r);
    PRINT_NAME_VAR(delta_dense_in_place.transpose());

    test_matrix_almost_equation(delta_dense_in_place, delta_dense);
}

int main(int argc, char *argv[])
{
    test_solvers();

    return 0;
}