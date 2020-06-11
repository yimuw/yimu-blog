#include <iostream>
#include <type_traits>
#include <cstdlib>
#include <cxxabi.h>
#include <memory>

#include <eigen3/Eigen/Core>

using namespace std;

template<int32_t COST>
struct C{
    using Type = C;
    static constexpr int32_t cost()
    {
        return COST;
    }

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
};

template <typename A, typename B>
struct Max{
    static constexpr int32_t cost() {
        return ((int32_t)A::cost() > (int32_t)B::cost()) ? (int32_t)A::cost() : (int32_t)B::cost();
    }

    // TODO: ::type::Type is ugly!
    typedef typename std::conditional<(A::cost() > B::cost()),A, B>::type::Type Type;

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
};

template <typename A, typename B>
struct Min{
    static constexpr int32_t cost() {
        return ((int32_t)A::cost() < (int32_t)B::cost()) ? (int32_t)A::cost() : (int32_t)B::cost();
    }

    typedef typename std::conditional<(A::cost() < B::cost()),A, B>::type::Type Type;

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
};

template<int32_t R, int32_t C>
using Matrix = Eigen::Matrix<double, R, C>;


template<int32_t ROWS, int32_t COLS>
struct Mat{
    // Used in struct Max/ struct Min
    using Type = Mat;
    using MatrixType = Matrix<ROWS, COLS>;

    MatrixType *mat_ {nullptr};

    Mat() = default;

    Mat(MatrixType &m)
        : mat_(&m)
    {}

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

    static constexpr bool is_leaf()
    {
        return true;
    }
};

template<typename A, typename B>
struct Prod
{
    // Used in struct Max/ struct Min
    using Type = Prod;
    using MatrixType = Matrix<A::rows(), B::cols()>;

    A a_;
    B b_;

    Prod() = default;

    Prod(A a, B b) 
        : a_(a), b_(b)
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

    static constexpr bool is_leaf()
    {
        return false;
    }
};


template<typename A, typename B>
typename Prod<A, B>::MatrixType eval_expression(Prod<A, B> E, void * p[])
{
    int idx = 0;
    return eval_expression(E, p, idx);
}

// TODO: using a type parameter to dispatch calls is inefficient. 
//       consider using template specification.
template<typename A, typename B>
typename Prod<A, B>::MatrixType eval_expression(Prod<A, B>, void * p[], int &idx)
{
    typename A::MatrixType m1 = eval_expression(A(), p, idx);
    typename B::MatrixType m2 = eval_expression(B(), p, idx);
    return m1 * m2;
}

template<typename M>
typename M::MatrixType eval_expression(M, void * p[], int &idx)
{
    M mat = *reinterpret_cast<M*>(p[idx++]);
    return mat.eval();
}

template<typename EigenM1,typename EigenM2,typename EigenM3>
auto prod(EigenM1 &m1, EigenM2 &m2, EigenM3 &m3)
{
    auto mm1 = Mat(m1);
    auto mm2 = Mat(m2);
    auto mm3 = Mat(m3);
    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using OptimalExpression = typename 
        Max<Prod<M1, Prod<M2, M3>>, Prod<Prod<M1, M2>, M3>>::Type;

    void * p[3] = {&mm1, &mm2, &mm3};
    OptimalExpression::print_type();
    return eval_expression(OptimalExpression(), p);
}

template<typename EigenM1,typename EigenM2,typename EigenM3,typename EigenM4>
auto prod(EigenM1 &m1, EigenM2 &m2, EigenM3 &m3, EigenM4 &m4)
{
    auto mm1 = Mat(m1);
    auto mm2 = Mat(m2);
    auto mm3 = Mat(m3);
    auto mm4 = Mat(m4);

    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using M4 = decltype(mm4);
    using OptimalExpression = typename 
        Max<Prod<M1,Prod<M2,Prod<M3,M4>>>,
        Max<Prod<M1,Prod<Prod<M2,M3>,M4>>,
        Max<Prod<Prod<M1,M2>,Prod<M3,M4>>,
        Max<Prod<Prod<M1,Prod<M2,M3>>,M4>,
        Prod<Prod<Prod<M1,M2>,M3>,M4>>>>>::Type;

    void * p[4] = {&mm1, &mm2, &mm3, &mm4};
    OptimalExpression::print_type();
    return eval_expression(OptimalExpression(), p);
}

template<typename EigenM1,typename EigenM2,typename EigenM3,typename EigenM4, typename EigenM5>
auto prod(EigenM1 &m1, EigenM2 &m2, EigenM3 &m3, EigenM4 &m4, EigenM5 &m5)
{
    auto mm1 = Mat(m1);
    auto mm2 = Mat(m2);
    auto mm3 = Mat(m3);
    auto mm4 = Mat(m4);
    auto mm5 = Mat(m5);

    using M1 = decltype(mm1);
    using M2 = decltype(mm2);
    using M3 = decltype(mm3);
    using M4 = decltype(mm4);
    using M5 = decltype(mm5);
    using OptimalExpression = typename
        Max<Prod<M1,Prod<M2,Prod<M3,Prod<M4,M5>>>>,
        Max<Prod<M1,Prod<M2,Prod<Prod<M3,M4>,M5>>>,
        Max<Prod<M1,Prod<Prod<M2,M3>,Prod<M4,M5>>>,
        Max<Prod<M1,Prod<Prod<M2,Prod<M3,M4>>,M5>>,
        Max<Prod<M1,Prod<Prod<Prod<M2,M3>,M4>,M5>>,
        Max<Prod<Prod<M1,M2>,Prod<M3,Prod<M4,M5>>>,
        Max<Prod<Prod<M1,M2>,Prod<Prod<M3,M4>,M5>>,
        Max<Prod<Prod<M1,Prod<M2,M3>>,Prod<M4,M5>>,
        Max<Prod<Prod<Prod<M1,M2>,M3>,Prod<M4,M5>>,
        Max<Prod<Prod<M1,Prod<M2,Prod<M3,M4>>>,M5>,
        Max<Prod<Prod<M1,Prod<Prod<M2,M3>,M4>>,M5>,
        Max<Prod<Prod<Prod<M1,M2>,Prod<M3,M4>>,M5>,
        Max<Prod<Prod<Prod<M1,Prod<M2,M3>>,M4>,M5>,
        Prod<Prod<Prod<Prod<M1,M2>,M3>,M4>,M5>>>>>>>>>>>>>>::Type;

    void * p[5] = {&mm1, &mm2, &mm3, &mm4, &mm5};
    OptimalExpression::print_type();
    return eval_expression(OptimalExpression(), p);
}


int main(int argc, char *argv[])
{
    cout << "C1::cost:" << C<1>::cost()  << endl;
    cout << "C2::cost:" << C<2>::cost()  << endl;

    {   
        constexpr int32_t max_val = Max<C<1>, C<2>>::cost();
        static_assert(max_val == 2);
        cout << "Cost<C<1>, C<2>>::cost(): " << max_val << endl;
    }

    {
        using MaxType = Max<C<3>, Max<C<1>, C<2>>>;
        constexpr int32_t max_val = MaxType::cost();
        static_assert(max_val == 3);
        cout << "Cost<C<3>, Cost<C<1>, C<2>>>: " << max_val << endl;
        MaxType::print_type();
        MaxType::Type::print_type();
    }

    {
        using M1 = Mat<2,20>;
        using M2 = Mat<20,4>;
        using M3 = Mat<4,8>;
        using P1 = Prod<Prod<M1, M2>, M3>;
        constexpr int32_t c = P1::cost();
        std::cout << "cp:" << c << std::endl;

        std::cout << "P1:" << P1::rows() << "," << P1::cols() << std::endl;
    }

    {
        using M1 = Mat<2,20>;
        using M2 = Mat<20,4>;
        using M3 = Mat<4,8>;
        using P1 = Prod<Prod<M1, M2>, M3>;
        using P2 = Prod<M1, Prod<M2, M3>>;

        std::cout << "p1.cost:" << P1::cost() << std::endl;
        std::cout << "p2.cost:" << P2::cost() << std::endl;

        using MaxP = Max<P1, P2>;
        std::cout << "MaxP.cost():" << MaxP::cost() << std::endl;
        MaxP::Type::print_type();

        using MinP = Min<P1, P2>;
        std::cout << "MaxP.cost():" << MinP::cost() << std::endl;
        MinP::Type::print_type();
    }


    {
        std::cout << "=========== expr 4 ===========" << std::endl;

        auto m1 = Matrix<5,4>();
        auto m2 = Matrix<4,4>();
        auto m3 = Matrix<4,4>();
        auto m4 = Matrix<4,5>();
        auto m5 = Matrix<5,10>();

        auto res3 = prod(m1, m2, m3);
        cout << "res3:" << res3 << std::endl;

        auto res4 = prod(m1, m2, m3, m4);
        cout << "res4:" << res4 << std::endl;

        auto res5 = prod(m1, m2, m3, m4, m5);
        cout << "res5:" << res5 << std::endl;
    }


    return 0;
}
