#include <iostream>

// #define PRINT_INTERMEDIATE_RESULTS
#include "optimal_matrix_mul.h"

#include "timming.h"

using namespace matrix_optimal_product;

int main(int argc, char* argv[])
{
    {
        std::cout << "=========== expr 4 ===========" << std::endl;

        auto m1 = Matrix<19, 9>();
        auto m2 = Matrix<9, 8>();
        auto m3 = Matrix<8, 7>();
        auto m4 = Matrix<7, 6>();
        auto m5 = Matrix<6, 6>();

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

    {
        std::cout << "============ timing =============" << std::endl;

        auto m1 = Matrix<100, 100>();
        m1 = m1.Random();
        auto m2 = Matrix<100, 50>();
        m2 = m2.Random();
        auto m3 = Matrix<50, 50>();
        m3 = m3.Random();
        auto m4 = Matrix<50, 6>();
        m4 = m4.Random();
        auto m5 = Matrix<6, 6>();
        m5 = m5.Random();

        constexpr int iters = 1e4;

        for(int i = 0; i < iters; ++i)
        {
            Timing t("eigen-mul");
            Eigen::MatrixXd res = m1 * m2 * m3 * m4 * m5;
        }

        for(int i = 0; i < iters; ++i)
        {
            Timing t("opti-mul");
            Eigen::MatrixXd res = prod(m1, m2, m3, m4, m5);
        }

        Timing::print_info();
    }

    return 0;
}
