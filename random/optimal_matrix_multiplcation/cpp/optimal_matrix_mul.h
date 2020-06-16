#pragma once

#include <iostream>
#include <type_traits>

#include <eigen3/Eigen/Core>

#include "core.h"
#include "expressions.h"
#include "utils.h"

namespace matrix_optimal_product {
template <typename EigenM1, typename EigenM2, typename EigenM3>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, bool verbose = false)
{
    auto me1 = MatExpression(m1);
    auto me2 = MatExpression(m2);
    auto me3 = MatExpression(m3);
    using M1 = decltype(me1);
    using M2 = decltype(me2);
    using M3 = decltype(me3);
    using MinExpression = MinExpression3<M1, M2, M3>;
    using OptimalExpression = typename MinExpression::Type;
    void* p[3] = { &m1, &m2, &m3 };

    if (verbose) {
        print_cost_for_min_expression(MinExpression());
        std::cout << "OptimalExpression:" << std::endl;
        OptimalExpression::print_expression();
        std::cout << std::endl;
        std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
    }

    return eval_expression(OptimalExpression(), p);
}

template <typename EigenM1, typename EigenM2, typename EigenM3, typename EigenM4>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, EigenM4& m4, bool verbose = false)
{
    auto me1 = MatExpression(m1);
    auto me2 = MatExpression(m2);
    auto me3 = MatExpression(m3);
    auto me4 = MatExpression(m4);

    using M1 = decltype(me1);
    using M2 = decltype(me2);
    using M3 = decltype(me3);
    using M4 = decltype(me4);
    using MinExpression = MinExpression4<M1, M2, M3, M4>;
    using OptimalExpression = typename MinExpression::Type;

    if (verbose) {
        print_cost_for_min_expression(MinExpression());
        std::cout << "OptimalExpression:" << std::endl;
        OptimalExpression::print_expression();
        std::cout << std::endl;
        std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
    }

    void* p[4] = { &m1, &m2, &m3, &m4 };
    return eval_expression(OptimalExpression(), p);
}

template <typename EigenM1, typename EigenM2, typename EigenM3, typename EigenM4, typename EigenM5>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, EigenM4& m4, EigenM5& m5, bool verbose = false)
{
    auto me1 = MatExpression(m1);
    auto me2 = MatExpression(m2);
    auto me3 = MatExpression(m3);
    auto me4 = MatExpression(m4);
    auto me5 = MatExpression(m5);

    using M1 = decltype(me1);
    using M2 = decltype(me2);
    using M3 = decltype(me3);
    using M4 = decltype(me4);
    using M5 = decltype(me5);
    using MinExpression = MinExpression5<M1, M2, M3, M4, M5>;
    using OptimalExpression = typename MinExpression::Type;

    if (verbose) {
        print_cost_for_min_expression(MinExpression());
        std::cout << "OptimalExpression:" << std::endl;
        OptimalExpression::print_expression();
        std::cout << std::endl;
        std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
    }
    void* p[5] = { &m1, &m2, &m3, &m4, &m5 };
    return eval_expression(OptimalExpression(), p);
}

template <typename EigenM1, typename EigenM2, typename EigenM3, typename EigenM4, typename EigenM5, typename EigenM6>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, EigenM4& m4, EigenM5& m5, EigenM6& m6, bool verbose = false)
{
    auto me1 = MatExpression(m1);
    auto me2 = MatExpression(m2);
    auto me3 = MatExpression(m3);
    auto me4 = MatExpression(m4);
    auto me5 = MatExpression(m5);
    auto me6 = MatExpression(m6);

    using M1 = decltype(me1);
    using M2 = decltype(me2);
    using M3 = decltype(me3);
    using M4 = decltype(me4);
    using M5 = decltype(me5);
    using M6 = decltype(me6);
    using MinExpression = MinExpression6<M1, M2, M3, M4, M5, M6>;
    using OptimalExpression = typename MinExpression::Type;

    if (verbose) {
        print_cost_for_min_expression(MinExpression());
        std::cout << "OptimalExpression:" << std::endl;
        OptimalExpression::print_expression();
        std::cout << std::endl;
        std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
    }
    void* p[6] = { &m1, &m2, &m3, &m4, &m5, &m6 };
    return eval_expression(OptimalExpression(), p);
}
}