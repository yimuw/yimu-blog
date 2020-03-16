#pragma once

#include <vector>

#include <eigen3/Eigen/Core>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector2d = Eigen::Vector2d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Matrix8d = Eigen::Matrix<double, 8, 8>;
using TimeStamp = double;
using TimeDuration = double;


struct RocketState
{
    // Using a single dt is a better option.
    // But it make index handling a little bit more complex.
    enum StateIndex : int16_t
    {
        i_dt = 0,
        i_position_x,
        i_position_y,
        i_velocity_x,
        i_velocity_y,
        i_heading,
        i_turning_rate,
        i_acceleration,
        STATE_SIZE
    };

    explicit RocketState(const double dt,
                         const double px,
                         const double py,
                         const double vel_x,
                         const double vel_y,
                         const double heading,
                         const double turning_rate,
                         const double acceleration)
    {
        variables << dt, px, py, vel_x, vel_y, heading, turning_rate, acceleration;
    }

    RocketState()
    {
    }

    static int variable_size()
    {
        return STATE_SIZE;
    }

    static int lcn_state_size()
    {
        // i_dt,
        // i_position_x,
        // i_position_y,
        // i_velocity_x,
        // i_velocity_y,
        // i_heading,
        return 6;
    }

    static int control_state_size()
    {
        // i_turning_rate,
        // i_acceleration,
        return 2;
    }

    // TODO: const version
    double& delta_time()
    {
        return variables[i_dt];
    }

    Eigen::Map<Vector6d> lcn_state()
    {
        return Eigen::Map<Vector6d>(variables.data());
    }

    Eigen::Map<Vector2d> cntl_state()
    {
        return Eigen::Map<Vector2d>(variables.data() + 6);
    }

    Vector6d lcn_state() const 
    {
        return variables.block<6,1>(0, 0);
    }

    Vector2d cntl_state() const
    {
        return variables.block<2,1>(6, 0);
    }

    Eigen::Map<Vector2d> position()
    {
        return Eigen::Map<Vector2d>(variables.data() + i_position_x);
    }

    Eigen::Map<Vector2d> velocity()
    {
        return Eigen::Map<Vector2d>(variables.data() + i_velocity_x);
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

    Vector2d velocity() const
    {
        return {variables(i_velocity_x), variables(i_velocity_y)};
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

    Vector8d variables = Vector8d::Zero();
};

struct Trajectory
{
    std::vector<RocketState> states;
};

struct NormalEqution
{
    NormalEqution() = default;

    NormalEqution(const size_t num_variables)
        : lhs(MatrixXd::Zero(num_variables, num_variables)),
            rhs(VectorXd::Zero(num_variables))
    {
    }
    // A.T * W * A
    MatrixXd lhs;

    // - A.T * b
    VectorXd rhs;
};