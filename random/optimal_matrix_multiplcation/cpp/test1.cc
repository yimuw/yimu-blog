#include <iostream>
#include <type_traits>
#include <cstdlib>
#include <cxxabi.h>
#include <memory>

#include <eigen3/Eigen/Core>

using namespace std;

template<int32_t COST>
struct C{
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

    typedef typename std::conditional<(A::cost() > B::cost()),A, B>::type Type;

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

    typedef typename std::conditional<(A::cost() < B::cost()),A, B>::type Type;

    static void print_type()
    {
        printf("%s\n", __PRETTY_FUNCTION__);
    }
};

template<int32_t R, int32_t C>
using Matrix = Eigen::Matrix<double, R, C>;


template<int32_t ROWS, int32_t COLS>
struct Mat{
    using MatrixType = Matrix<ROWS, COLS>;

    MatrixType mat_;

    Mat() = default;

    Mat(MatrixType &m)
        : mat_(m)
    {}

    MatrixType eval()
    {
        return mat_;
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
        auto m1 = Matrix<2, 20>();
        auto m2 = Matrix<20, 4>();
        auto m3 = Matrix<4, 8>();

        auto mm1 = Mat(m1);
        auto mm2 = Mat(m2);
        auto mm3 = Mat(m3);

        auto p1 = Prod(mm1, mm2);
        auto p2 = Prod(p1, mm3);

        auto res = p2.eval();
        cout << "res.rc:" << res.rows() << "," << res.cols() << endl;

        auto p = Prod(mm1, Prod{mm2, mm3});
        cout << p.eval() << endl;
    }

    {
        using M1 = Mat<2,3>;
        using M2 = Mat<3,4>;
        using M3 = Mat<4,3>;

        using P1 = Prod<Prod<M1, M2>, M3>;
        using P2 = Prod<M1, Prod<M2, M3>>;

        auto m1 = Matrix<2, 3>();
        m1 << 1,2,3,4,5,6;
        auto m2 = Matrix<3, 4>();
        m2 << 1,2,3,4,5,6,7,8,9,10,11,12;
        auto m3 = Matrix<4, 3>();
        m3 << 1,2,3,4,5,6,7,8,9,10,11,12;

        auto mm1 = M1(m1);
        auto mm2 = M2(m2);
        auto mm3 = M3(m3);

        void * p[3] = {&mm1, &mm2, &mm3};

        int idx = 0;
        auto p1_res = eval_expression(P1(), p, idx);
        cout << "p1_res: " << p1_res << std::endl;

        idx = 0;
        auto p2_res = eval_expression(P2(), p, idx);
        cout << "p2_res: " << p2_res << std::endl;

        using MaxP = Max<P1, P2>;
        idx = 0;
        auto p3_res = eval_expression(MaxP::Type(), p, idx);
        cout << "p3_res: " << p3_res << std::endl;

    }
    return 0;
}