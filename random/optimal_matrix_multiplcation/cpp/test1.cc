#include <iostream>

#define PRINT_INTERMEDIATE_RESULTS
#include "optimal_matrix_mul.h"

using namespace matrix_optimal_product;

int main(int argc, char* argv[])
{
    {
        using M1 = MatExpression<2, 20>;
        using M2 = MatExpression<20, 4>;
        using M3 = MatExpression<4, 8>;
        using P1 = ProdExpression<ProdExpression<M1, M2>, M3>;
        constexpr int32_t c = P1::cost();
        std::cout << "cp:" << c << std::endl;

        std::cout << "P1:" << P1::rows() << "," << P1::cols() << std::endl;
    }

    {
        using M1 = MatExpression<2, 20>;
        using M2 = MatExpression<20, 4>;
        using M3 = MatExpression<4, 8>;
        using P1 = ProdExpression<ProdExpression<M1, M2>, M3>;
        using P2 = ProdExpression<M1, ProdExpression<M2, M3>>;

        std::cout << "p1.cost:" << P1::cost() << std::endl;
        std::cout << "p2.cost:" << P2::cost() << std::endl;

        using MinP = Min<P1, P2>;
        std::cout << "MaxP.cost():" << MinP::cost() << std::endl;
        MinP::Type::print_type();
    }

    {
        std::cout << "=========== expr 4 ===========" << std::endl;

        auto m1 = Matrix<3, 4>();
        auto m2 = Matrix<4, 8>();
        auto m3 = Matrix<8, 6>();
        auto m4 = Matrix<6, 8>();
        auto m5 = Matrix<8, 10>();

        std::cout << "============= prod 3 =================" << std::endl;
        auto res3 = prod(m1, m2, m3);
        std::cout << "res3:" << res3 << std::endl;

        std::cout << "============= prod 4 =================" << std::endl;
        auto res4 = prod(m1, m2, m3, m4);
        std::cout << "res4:" << res4 << std::endl;

        std::cout << "============= prod 5 =================" << std::endl;
        auto res5 = prod(m1, m2, m3, m4, m5);
        std::cout << "res5:" << res5 << std::endl;
    }

    return 0;
}
