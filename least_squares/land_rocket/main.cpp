#include <iostream>

#include "rocket_landing_planner.h"


int main(int argc, char *argv[])
{
    //  dt, x, y, vel, heading, turn_rate, accl
    RocketState start_state(1., 0., 0., 0., 0., 0., 0.);
    RocketState end_state(1., 100., 100., 0., 0., 0., 0.);
    int steps = 100;

    RocketLandingPlanner rocket_landing(start_state, end_state, steps);
    rocket_landing.solve();

    return 0;
}