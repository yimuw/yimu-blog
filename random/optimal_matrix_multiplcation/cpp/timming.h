#pragma once

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <vector>

struct Timing {
    using timing_map = std::map<std::string, std::vector<double>>;
    using timing_result = std::map<std::string, double>;

    inline static timing_map name_to_times_;

    static std::map<std::string, double> print_info()
    {
        timing_result result;
        for (auto& p : name_to_times_) {
            double sum = std::accumulate(p.second.begin(), p.second.end(), 0.0);
            double mean = sum / p.second.size();

            double sq_sum = std::inner_product(p.second.begin(), p.second.end(), p.second.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / p.second.size() - mean * mean);
            std::cout << p.first << "  mean(ms): " << mean * 1e3 << " stdev(ms):" << stdev * 1e3 << std::endl;
            
            result.insert({p.first, mean * 1e3});
        }
        return result;
    }

    Timing(const std::string& name)
        : name_(name)
        , start_(std::chrono::high_resolution_clock::now())
    {
    }

    ~Timing()
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start_;

        name_to_times_[name_].push_back(diff.count());
    }

    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};