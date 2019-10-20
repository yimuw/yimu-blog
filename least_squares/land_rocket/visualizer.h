#pragma once

#include "types.h"
#include "third_party/matplotlib-cpp.h"

namespace plt = matplotlibcpp;


class PythonmatplotVisualizer
{
public:
    void plot_trajectory(const Trajectory &trajectory)
    {
        std::vector<double> dts;
        std::vector<double> pxs;
        std::vector<double> pys;
        std::vector<double> headings;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        std::vector<double> turning_rates;

        assert(trajectory.states.size() > 0);

        for(const auto &s : trajectory.states)
        {
            dts.push_back(s.delta_time());
            pxs.push_back(s.position().x());
            pys.push_back(s.position().y());
            headings.push_back(s.heading());
            velocities.push_back(s.velocity());
            accelerations.push_back(s.acceleration());
            turning_rates.push_back(s.turning_rate());
        }

        std::cout << "dts.size(): " << dts.size() << std::endl;
        std::vector<double> times(dts.size());
        std::partial_sum(dts.begin(), dts.end(), times.begin());

        assert(times.size() == pxs.size());
        plt::subplot(3,2,1);
        plt::title("time vs position_x");
        plt::plot(times, pxs);

        plt::subplot(3,2,2);
        plt::title("time vs position_y");
        plt::plot(times, pys);

        plt::subplot(3,2,3);
        plt::title("time vs heading");
        plt::plot(times, headings);

        plt::subplot(3,2,4);
        plt::title("time vs velocity");
        plt::plot(times, velocities);

        plt::subplot(3,2,5);
        plt::title("time vs accelerations");
        plt::plot(times, accelerations);

        plt::subplot(3,2,6);
        plt::title("time vs turning_rates");
        plt::plot(times, turning_rates);

        plt::show();
    }
private:

};