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
        std::vector<double> vx;
        std::vector<double> vy;
        std::vector<double> accelerations;
        std::vector<double> turning_rates;

        assert(trajectory.states.size() > 0);

        for(const auto &s : trajectory.states)
        {
            dts.push_back(s.delta_time());
            pxs.push_back(s.position().x());
            pys.push_back(s.position().y());
            headings.push_back(s.heading());
            vx.push_back(s.velocity().x());
            vy.push_back(s.velocity().y());
            accelerations.push_back(s.acceleration());
            turning_rates.push_back(s.turning_rate());
        }

        std::cout << "dts.size(): " << dts.size() << std::endl;
        std::vector<double> times(dts.size());
        std::partial_sum(dts.begin(), dts.end(), times.begin());

        assert(times.size() == pxs.size());
        plt::subplot(3,3,1);
        plt::title("time vs position_x");
        plt::plot(times, pxs);

        plt::subplot(3,3,2);
        plt::title("time vs position_y");
        plt::plot(times, pys);

        plt::subplot(3,3,3);
        plt::title("time vs heading");
        plt::plot(times, headings);

        plt::subplot(3,3,4);
        plt::title("time vs vx");
        plt::plot(times, vx);

        plt::subplot(3,3,5);
        plt::title("time vs vy");
        plt::plot(times, vy);

        plt::subplot(3,3,6);
        plt::title("time vs accelerations");
        plt::plot(times, accelerations);

        plt::subplot(3,3,7);
        plt::title("time vs turning_rates");
        plt::plot(times, turning_rates);

        plt::subplot(3,3,8);
        plt::title("time");
        plt::plot(times);

        plt::show();
    }

    void spy_matrix(const Eigen::MatrixXd &mat)
    {
        std::vector<std::vector<double>> vvmat(mat.rows(), std::vector<double>(mat.cols()));
        for(int col = 0; col < mat.cols(); ++col)
        {
            for(int row = 0; row < mat.rows(); ++row)
            {
                vvmat[row][col] = mat(row, col);
            }
        }

        plt::spy(vvmat, {});
        plt::title("spy");
        plt::show();
    }
private:

};