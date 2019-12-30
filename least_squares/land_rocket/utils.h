#pragma once

#include <iomanip>
#include <algorithm>
#include <iostream>
#include <chrono>

#include "least_square_problem.h"


#define PRINT_NAME_VAR(var) std::cout << std::setprecision(5) << #var << " :\n" << var << std::endl

void print_variables(Trajectory trajectory, bool with_prediction = false)
{
    for(size_t state_i = 0; state_i < trajectory.states.size(); ++state_i)
    {
        std::cout << "state " << state_i << " :" 
            << trajectory.states.at(state_i).variables.transpose() << std::endl;
        if(with_prediction)
        {
            std::cout << "pred  " << state_i << " :" 
                << RocketMotionModel().motion(trajectory.states.at(state_i)).variables.transpose() << std::endl;
        }
    }
}

VectorXd trajectory_to_vectorXd(const Trajectory &trajectory)
{
    const int num_states = trajectory.states.size();
    VectorXd result(num_states * RocketState::STATE_SIZE);
    for(int state_i = 0; state_i < num_states; ++state_i)
    {
        result.segment(state_i * RocketState::STATE_SIZE, RocketState::STATE_SIZE) = trajectory.states.at(state_i).variables;
    }
    return result;
}

Trajectory update_primal_variables(const VectorXd &delta, 
                                   const double step_size,
                                   const Trajectory &trajectory)
{
    const int num_states = trajectory.states.size();
    Trajectory result = trajectory;
    for(int state_i = 0; state_i < num_states; ++state_i)
    {
        const VectorXd delta_i = delta.segment(state_i * RocketState::STATE_SIZE, RocketState::STATE_SIZE);
        result.states.at(state_i).variables += step_size * delta_i;
    }
    return result;
}

#define PROFILER

struct ScopeProfiler
{
    using time_point = std::chrono::steady_clock::time_point;

#ifdef PROFILER
    ScopeProfiler(const std::string &info)
        : info_(info)
    {
        begin_ = std::chrono::steady_clock::now();
    }

    ~ScopeProfiler()
    {
        time_point end = std::chrono::steady_clock::now();
        auto elapsed_msec = std::chrono::duration_cast<std::chrono::microseconds>(end - begin_).count();
        std::cout << "Profiler: " << info_ << " msec:" << elapsed_msec / 1000. << std::endl;
    }
#else
    
#endif

    std::string info_;
    time_point begin_;
};