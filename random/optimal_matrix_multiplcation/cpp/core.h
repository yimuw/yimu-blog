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

    typedef typename std::conditional<(A::cost() < B::cost()), A, B>::type::Type Type;

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

    MatrixType* mat_{ nullptr };

    MatExpression() = default;

    MatExpression(MatrixType& m)
        : mat_(&m)
    {
    }

    MatrixType eval()
    {
        assert(mat_);
        return *mat_;
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

    A a_;
    B b_;

    ProdExpression() = default;

    ProdExpression(A a, B b)
        : a_(a)
        , b_(b)
    {
    }

    MatrixType eval()
    {
        return a_.eval() * b_.eval();
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
    M mat = *reinterpret_cast<M*>(p[idx++]);
    return mat.eval();
}

}