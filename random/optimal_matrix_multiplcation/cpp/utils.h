#pragma once

#include <iostream>

#include "core.h"

namespace matrix_optimal_product {

template <typename E>
void print_cost_for_min_expression(E)
{
    std::cout << "Expression: " << std::endl;
    E::print_expression();
    std::cout << std::endl;
    std::cout << "cost:" << E::cost() << std::endl;
}

template <typename A, typename B>
void print_cost_for_min_expression(Min<A, B>)
{
    std::cout << "Expression: ";
    A::print_expression();
    std::cout << std::endl;
    std::cout << "cost:" << A::cost() << std::endl;

    print_cost_for_min_expression(B());
}
}