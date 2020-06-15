#include <iostream>

#include "optimal_matrix_mul.h"
#include "timming.h"

using namespace matrix_optimal_product;

constexpr double EPSILON = 1e-8;

void testCompilationTime()
{
    std::cout << "============ testCompilationTime =============" << std::endl;
    auto m1 = Matrix<19, 9>();
    auto m2 = Matrix<9, 8>();
    auto m3 = Matrix<8, 7>();

    auto me1 = MatExpression(m1);
    auto me2 = MatExpression(m2);
    auto me3 = MatExpression(m3);
    using M1 = decltype(me1);
    using M2 = decltype(me2);
    using M3 = decltype(me3);
    using MinExpression = MinExpression3<M1, M2, M3>;

    print_cost_for_min_expression(MinExpression());

    using OptimalExpression = typename MinExpression::Type;

    std::cout << "OptimalExpression:" << std::endl;
    OptimalExpression::print_expression();
    std::cout << std::endl;
    std::cout << "optimal cost:" << OptimalExpression::cost() << std::endl;

    // make sure the cost is computed at compilation time.
    static_assert(OptimalExpression::cost() == 1701, "cost is not computed at complie time!");
    std::cout << "cost computed at compilation time" << std::endl;
}

void testCorrectness()
{
    std::cout << "=========== testCorrectness ===========" << std::endl;

    auto m1 = Matrix<19, 9>();
    auto m2 = Matrix<9, 8>();
    auto m3 = Matrix<8, 7>();
    auto m4 = Matrix<7, 6>();
    auto m5 = Matrix<6, 6>();
    auto m6 = Matrix<6, 10>();

    // TODO: using Matrix<6, 6>::Random makes the template deduction failed.
    m1 = m1.Random();
    m2 = m2.Random();
    m3 = m3.Random();
    m4 = m4.Random();
    m5 = m5.Random();
    m6 = m6.Random();

    auto res3 = prod(m1, m2, m3);
    auto gt_res3 = m1 * m2 * m3;
    assert(std::abs((res3 - gt_res3).norm()) < EPSILON && "matrix mul 3 failed");
    std::cout << "matrix mul 3 passed" << std::endl;

    auto res4 = prod(m1, m2, m3, m4);
    auto gt_res4 = m1 * m2 * m3 * m4;
    assert(std::abs((res4 - gt_res4).norm()) < EPSILON && "matrix mul 4 failed");
    std::cout << "matrix mul 4 passed" << std::endl;

    auto res5 = prod(m1, m2, m3, m4, m5);
    auto gt_res5 = m1 * m2 * m3 * m4 * m5;
    assert(std::abs((res5 - gt_res5).norm()) < EPSILON && "matrix mul 5 failed");
    std::cout << "matrix mul 5 passed" << std::endl;

    auto res6 = prod(m1, m2, m3, m4, m5, m6);
    auto gt_res6 = m1 * m2 * m3 * m4 * m5 * m6;
    assert(std::abs((res6 - gt_res6).norm()) < EPSILON && "matrix mul 6 failed");
    std::cout << "matrix mul 6 passed" << std::endl;
}

void profile()
{
    std::cout << "============ timing =============" << std::endl;

    auto m1 = Matrix<50, 50>();
    m1 = m1.Random();
    auto m2 = Matrix<50, 20>();
    m2 = m2.Random();
    auto m3 = Matrix<20, 50>();
    m3 = m3.Random();
    auto m4 = Matrix<50, 6>();
    m4 = m4.Random();
    auto m5 = Matrix<6, 6>();
    m5 = m5.Random();

    // This is not is best way to profile a function.
    constexpr int iters = 1e5;

    for (int i = 0; i < iters; ++i) {
        Timing t("opti-mul");
        Eigen::MatrixXd res = prod(m1, m2, m3, m4, m5);
    }

    for (int i = 0; i < iters; ++i) {
        Timing t("eigen-mul");
        Eigen::MatrixXd res = m1 * m2 * m3 * m4 * m5;
    }

    std::map<std::string, double> timing_info = Timing::print_info();

    std::cout << "optimal mul speed-up: " 
        << timing_info.at("eigen-mul") / timing_info.at("opti-mul") << std::endl;

    // Compute predicted speed-up by muls.
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
    std::cout << "# optimal muls:" << OptimalExpression::cost() << std::endl;
    using DefaultExpression = Prod<Prod<Prod<Prod<M1, M2>, M3>, M4>, M5>;
    std::cout << "# default muls:" << DefaultExpression::cost() << std::endl;

    std::cout << "predicted speed-up by # muls: "
              << DefaultExpression::cost() / static_cast<double>(OptimalExpression::cost()) << std::endl;
}

int main(int argc, char* argv[])
{
    testCompilationTime();

    testCorrectness();

    profile();

    return 0;
}
