#pragma once

#include <vector>

#include <eigen3/Eigen/Core>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector2d = Eigen::Vector2d;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using TimeStamp = double;
using TimeDuration = double;

struct RocketState
{
    enum StateIndex
    {
        i_dt = 0,
        i_position_x,
        i_position_y,
        i_velocity,
        i_heading,
        i_turning_rate,
        i_acceleration,
        STATE_SIZE
    };

    int variable_size() const
    {
        return STATE_SIZE;
    }

    // TODO: const version
    double& delta_time()
    {
        return variables[i_dt];
    }

    Eigen::Map<Vector2d> position()
    {
        return Eigen::Map<Vector2d>(variables.data() + i_position_x);
    }

    double& velocity()
    {
        return variables[i_velocity];
    }

    double& heading()
    {
        return variables[i_heading];
    }

    double& turning_rate()
    {
        return variables[i_turning_rate];
    }

    double& acceleration()
    {
        return variables[i_acceleration];
    }

    // TODO: const version
    const double& delta_time() const
    {
        return variables[i_dt];
    }

    Vector2d position() const
    {
        return {variables(i_position_x), variables(i_position_y)};
    }

    const double& velocity() const
    {
        return variables[i_velocity];
    }

    const double& heading() const
    {
        return variables[i_heading];
    }

    const double& turning_rate() const
    {
        return variables[i_turning_rate];
    }

    const double& acceleration() const
    {
        return variables[i_acceleration];
    }

    Vector7d variables;
};

struct Trajectory
{
    std::vector<RocketState> states;
};

