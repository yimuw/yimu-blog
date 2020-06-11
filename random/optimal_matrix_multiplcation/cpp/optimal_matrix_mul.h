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

#ifdef PRINT_INTERMEDIATE_RESULTS
    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
#endif
};

#ifdef PRINT_INTERMEDIATE_RESULTS
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
#endif

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

#ifdef PRINT_INTERMEDIATE_RESULTS
    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }

    static void print_expression()
    {
        std::cout << "[mat,size:" << ROWS << "," << COLS << "]";
    }
#endif
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
#ifdef PRINT_INTERMEDIATE_RESULTS
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
#endif
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

template <typename EigenM1, typename EigenM2, typename EigenM3>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3)
{
    auto mm1 = MatExpression(m1);
    auto mm2 = MatExpression(m2);
    auto mm3 = MatExpression(m3);
    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using MinExpression = Min<ProdExpression<M1, ProdExpression<M2, M3>>, ProdExpression<ProdExpression<M1, M2>, M3>>;
    using OptimalExpression = typename MinExpression::Type;
    void* p[3] = { &mm1, &mm2, &mm3 };

#ifdef PRINT_INTERMEDIATE_RESULTS
    print_cost_for_min_expression(MinExpression());
    std::cout << "OptimalExpression:" << std::endl;
    OptimalExpression::print_expression();
    std::cout << std::endl;
    std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
#endif
    return eval_expression(OptimalExpression(), p);
}

template <typename EigenM1, typename EigenM2, typename EigenM3, typename EigenM4>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, EigenM4& m4)
{
    auto mm1 = MatExpression(m1);
    auto mm2 = MatExpression(m2);
    auto mm3 = MatExpression(m3);
    auto mm4 = MatExpression(m4);

    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using M4 = decltype(mm4);
    using MinExpression = Min<ProdExpression<M1, ProdExpression<M2, ProdExpression<M3, M4>>>,
        Min<ProdExpression<M1, ProdExpression<ProdExpression<M2, M3>, M4>>,
            Min<ProdExpression<ProdExpression<M1, M2>, ProdExpression<M3, M4>>,
                Min<ProdExpression<ProdExpression<M1, ProdExpression<M2, M3>>, M4>,
                    ProdExpression<ProdExpression<ProdExpression<M1, M2>, M3>, M4>>>>>;
    using OptimalExpression = typename MinExpression::Type;

#ifdef PRINT_INTERMEDIATE_RESULTS
    print_cost_for_min_expression(MinExpression());
    std::cout << "OptimalExpression:" << std::endl;
    OptimalExpression::print_expression();
    std::cout << std::endl;
    std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
#endif

    void* p[4] = { &mm1, &mm2, &mm3, &mm4 };
    return eval_expression(OptimalExpression(), p);
}

template <typename EigenM1, typename EigenM2, typename EigenM3, typename EigenM4, typename EigenM5>
auto prod(EigenM1& m1, EigenM2& m2, EigenM3& m3, EigenM4& m4, EigenM5& m5)
{
    auto mm1 = MatExpression(m1);
    auto mm2 = MatExpression(m2);
    auto mm3 = MatExpression(m3);
    auto mm4 = MatExpression(m4);
    auto mm5 = MatExpression(m5);

    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using M4 = decltype(mm4);
    using M5 = decltype(mm5);
    using MinExpression = Min<ProdExpression<M1, ProdExpression<M2, ProdExpression<M3, ProdExpression<M4, M5>>>>,
        Min<ProdExpression<M1, ProdExpression<M2, ProdExpression<ProdExpression<M3, M4>, M5>>>,
            Min<ProdExpression<M1, ProdExpression<ProdExpression<M2, M3>, ProdExpression<M4, M5>>>,
                Min<ProdExpression<M1, ProdExpression<ProdExpression<M2, ProdExpression<M3, M4>>, M5>>,
                    Min<ProdExpression<M1, ProdExpression<ProdExpression<ProdExpression<M2, M3>, M4>, M5>>,
                        Min<ProdExpression<ProdExpression<M1, M2>, ProdExpression<M3, ProdExpression<M4, M5>>>,
                            Min<ProdExpression<ProdExpression<M1, M2>, ProdExpression<ProdExpression<M3, M4>, M5>>,
                                Min<ProdExpression<ProdExpression<M1, ProdExpression<M2, M3>>, ProdExpression<M4, M5>>,
                                    Min<ProdExpression<ProdExpression<ProdExpression<M1, M2>, M3>, ProdExpression<M4, M5>>,
                                        Min<ProdExpression<ProdExpression<M1, ProdExpression<M2, ProdExpression<M3, M4>>>, M5>,
                                            Min<ProdExpression<ProdExpression<M1, ProdExpression<ProdExpression<M2, M3>, M4>>, M5>,
                                                Min<ProdExpression<ProdExpression<ProdExpression<M1, M2>, ProdExpression<M3, M4>>, M5>,
                                                    Min<ProdExpression<ProdExpression<ProdExpression<M1, ProdExpression<M2, M3>>, M4>, M5>,
                                                        ProdExpression<ProdExpression<ProdExpression<ProdExpression<M1, M2>, M3>, M4>, M5>>>>>>>>>>>>>>;
    using OptimalExpression = typename MinExpression::Type;

#ifdef PRINT_INTERMEDIATE_RESULTS
    print_cost_for_min_expression(MinExpression());
    std::cout << "OptimalExpression:" << std::endl;
    OptimalExpression::print_expression();
    std::cout << std::endl;
    std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;
#endif
    void* p[5] = { &mm1, &mm2, &mm3, &mm4, &mm5 };
    return eval_expression(OptimalExpression(), p);
}
}