#include <iostream>

#include "rocket_landing_planner.h"
#include "visualizer.h"


int main(int argc, char *argv[])
{
    // //  dt, x, y, vx, vy, heading, turn_rate, accl
    // int steps = 100;
    // RocketState start_state(0.1, -10., 10., 1., 0., M_PI / 10., 0., 0);
    // RocketState end_state  (0.1, 0., 0., 0., 0., M_PI / 2.,  0., 0.);

    // int steps = 100;
    // RocketState start_state(0.1, 0., 0., 0., 0., M_PI / 2., 0., 0);
    // RocketState end_state  (0.1, 0., 10., 0., 0., M_PI / 2.,  0., 0.);

    int steps = 50;
    RocketState start_state(0.1, 0., 0., 0., 0., M_PI / 4., 0., 0);
    RocketState end_state  (0.1, 10., 10., 0., 0., M_PI / 2.,  0., 0.);
    RocketLandingPlanner rocket_landing(start_state, end_state, steps);

    if (argc > 1) {
        std::string solver_type = argv[1];
        if (solver_type == "s") rocket_landing.config_.solve_type = "sparse";
        else if (solver_type == "t") rocket_landing.config_.solve_type = "structural";
        else if (solver_type == "d") rocket_landing.config_.solve_type = "ddp_unconstrained";
        else if (solver_type == "n") rocket_landing.config_.solve_type = "naive";
        else if (solver_type == "g") rocket_landing.config_.solve_type = "gradient_desent";
    }
    std::cout << "solver type :" << rocket_landing.config_.solve_type << std::endl;

    {
        ScopeProfiler p("solve");
        rocket_landing.solve();
    }
    Trajectory result_traj = rocket_landing.get_trajectory();

    // PythonmatplotVisualizer visualizer;
    // visualizer.plot_trajectory(result_traj);
    // visualizer.video(result_traj);

    save_trajectory_as_csv("result.csv", result_traj);

    return 0;
}