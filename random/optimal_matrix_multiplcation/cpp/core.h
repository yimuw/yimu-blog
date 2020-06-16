#pragma once

#include <iostream>
#include <type_traits>

#include <eigen3/Eigen/Core>

namespace matrix_optimal_product {
template <typename A, typename B>
struct Min {
    static constexpr int32_t cost()
    {
        return ((int32_t)A::cost() < (int32_t)B::cost()) ? (int32_t)A::cost() : (int32_t)B::cost();
    }

    using Type = typename std::conditional<(A::cost() < B::cost()), A, B>::type::Type;

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
};


template <int32_t R, int32_t C>
using Matrix = Eigen::Matrix<double, R, C>;

template <int32_t ROWS, int32_t COLS>
struct MatExpression {
    // Used in struct Max/ struct Min
    using Type = MatExpression;
    using MatrixType = Matrix<ROWS, COLS>;

    MatExpression() = default;

    MatExpression(MatrixType&)
    {
    }

    static constexpr int32_t cost()
    {
        return 0;
    }

    static constexpr int32_t rows()
    {
        return ROWS;
    }

    static constexpr int32_t cols()
    {
        return COLS;
    }

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }

    static void print_expression()
    {
        std::cout << "[mat,size:" << ROWS << "," << COLS << "]";
    }
};

template <typename A, typename B>
struct ProdExpression {
    // Used in struct Max/ struct Min
    using Type = ProdExpression;
    using MatrixType = Matrix<A::rows(), B::cols()>;

    ProdExpression() = default;

    ProdExpression(A, B)
    {
    }

    static constexpr int32_t cost()
    {
        return A::cost() + B::cost() + A::rows() * A::cols() * B::cols();
    }

    static constexpr int32_t rows()
    {
        return A::rows();
    }

    static constexpr int32_t cols()
    {
        return B::cols();
    }

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }

    static void print_expression()
    {
        std::cout << "(";
        A::print_expression();
        std::cout << " * ";
        B::print_expression();
        std::cout << ")";
    }
};

template <typename A, typename B>
typename ProdExpression<A, B>::MatrixType eval_expression(ProdExpression<A, B> E, void* p[])
{
    int idx = 0;
    return eval_expression(E, p, idx);
}

// TODO: using a type parameter to dispatch calls is inefficient.
//       consider using template specification.
template <typename A, typename B>
typename ProdExpression<A, B>::MatrixType eval_expression(ProdExpression<A, B>, void* p[], int& idx)
{
    typename A::MatrixType m1 = eval_expression(A(), p, idx);
    typename B::MatrixType m2 = eval_expression(B(), p, idx);
    return m1 * m2;
}

template <typename M>
typename M::MatrixType eval_expression(M, void* p[], int& idx)
{
    using EigenMatType = typename M::MatrixType;
    return *reinterpret_cast<EigenMatType*>(p[idx++]);
}

}