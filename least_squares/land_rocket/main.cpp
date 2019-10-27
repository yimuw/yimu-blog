#include <iostream>

#include "rocket_landing_planner.h"
#include "visualizer.h"


int main(int argc, char *argv[])
{
    // //  dt, x, y, vx, vy, heading, turn_rate, accl
    int steps = 50;
    RocketState start_state(0.1, -2., 10., 0., 0., M_PI / 2., 0., 0);
    RocketState end_state  (0.1, 0., 0., 0., 0., M_PI / 2.,  0., 0.);

    RocketLandingPlanner rocket_landing(start_state, end_state, steps);
    rocket_landing.solve();
    Trajectory result_traj = rocket_landing.get_trajectory();

    PythonmatplotVisualizer visualizer;
    visualizer.plot_trajectory(result_traj);

    return 0;
}