#pragma once

#include "optimal_matrix_mul.h"

namespace matrix_optimal_product {
template<typename A, typename B>
using Prod = ProdExpression<A, B>;

template<typename M1, typename M2, typename M3>
using MinExpression3 = Min<Prod<M1, Prod<M2, M3>>, Prod<Prod<M1, M2>, M3>>;

template<typename M1, typename M2, typename M3, typename M4>
using MinExpression4 = Min<Prod<M1, Prod<M2, Prod<M3, M4>>>,
    Min<Prod<M1, Prod<Prod<M2, M3>, M4>>,
        Min<Prod<Prod<M1, M2>, Prod<M3, M4>>,
            Min<Prod<Prod<M1, Prod<M2, M3>>, M4>,
                Prod<Prod<Prod<M1, M2>, M3>, M4>>>>>;

template<typename M1, typename M2, typename M3, typename M4, typename M5>
using MinExpression5 = Min<Prod<M1, Prod<M2, Prod<M3, Prod<M4, M5>>>>,
    Min<Prod<M1, Prod<M2, Prod<Prod<M3, M4>, M5>>>,
        Min<Prod<M1, Prod<Prod<M2, M3>, Prod<M4, M5>>>,
            Min<Prod<M1, Prod<Prod<M2, Prod<M3, M4>>, M5>>,
                Min<Prod<M1, Prod<Prod<Prod<M2, M3>, M4>, M5>>,
                    Min<Prod<Prod<M1, M2>, Prod<M3, Prod<M4, M5>>>,
                        Min<Prod<Prod<M1, M2>, Prod<Prod<M3, M4>, M5>>,
                            Min<Prod<Prod<M1, Prod<M2, M3>>, Prod<M4, M5>>,
                                Min<Prod<Prod<Prod<M1, M2>, M3>, Prod<M4, M5>>,
                                    Min<Prod<Prod<M1, Prod<M2, Prod<M3, M4>>>, M5>,
                                        Min<Prod<Prod<M1, Prod<Prod<M2, M3>, M4>>, M5>,
                                            Min<Prod<Prod<Prod<M1, M2>, Prod<M3, M4>>, M5>,
                                                Min<Prod<Prod<Prod<M1, Prod<M2, M3>>, M4>, M5>,
                                                    Prod<Prod<Prod<Prod<M1, M2>, M3>, M4>, M5>>>>>>>>>>>>>>;

template<typename M1, typename M2, typename M3, typename M4, typename M5, typename M6>
using MinExpression6 = Min<Prod<M1, Prod<M2, Prod<M3, Prod<M4, Prod<M5, M6>>>>>,
    Min<Prod<M1, Prod<M2, Prod<M3, Prod<Prod<M4, M5>, M6>>>>,
        Min<Prod<M1, Prod<M2, Prod<Prod<M3, M4>, Prod<M5, M6>>>>,
            Min<Prod<M1, Prod<M2, Prod<Prod<M3, Prod<M4, M5>>, M6>>>,
                Min<Prod<M1, Prod<M2, Prod<Prod<Prod<M3, M4>, M5>, M6>>>,
                    Min<Prod<M1, Prod<Prod<M2, M3>, Prod<M4, Prod<M5, M6>>>>,
                        Min<Prod<M1, Prod<Prod<M2, M3>, Prod<Prod<M4, M5>, M6>>>,
                            Min<Prod<M1, Prod<Prod<M2, Prod<M3, M4>>, Prod<M5, M6>>>,
                                Min<Prod<M1, Prod<Prod<Prod<M2, M3>, M4>, Prod<M5, M6>>>,
                                    Min<Prod<M1, Prod<Prod<M2, Prod<M3, Prod<M4, M5>>>, M6>>,
                                        Min<Prod<M1, Prod<Prod<M2, Prod<Prod<M3, M4>, M5>>, M6>>,
                                            Min<Prod<M1, Prod<Prod<Prod<M2, M3>, Prod<M4, M5>>, M6>>,
                                                Min<Prod<M1, Prod<Prod<Prod<M2, Prod<M3, M4>>, M5>, M6>>,
                                                    Min<Prod<M1, Prod<Prod<Prod<Prod<M2, M3>, M4>, M5>, M6>>,
                                                        Min<Prod<Prod<M1, M2>, Prod<M3, Prod<M4, Prod<M5, M6>>>>,
                                                            Min<Prod<Prod<M1, M2>, Prod<M3, Prod<Prod<M4, M5>, M6>>>,
                                                                Min<Prod<Prod<M1, M2>, Prod<Prod<M3, M4>, Prod<M5, M6>>>,
                                                                    Min<Prod<Prod<M1, M2>, Prod<Prod<M3, Prod<M4, M5>>, M6>>,
                                                                        Min<Prod<Prod<M1, M2>, Prod<Prod<Prod<M3, M4>, M5>, M6>>,
                                                                            Min<Prod<Prod<M1, Prod<M2, M3>>, Prod<M4, Prod<M5, M6>>>,
                                                                                Min<Prod<Prod<M1, Prod<M2, M3>>, Prod<Prod<M4, M5>, M6>>,
                                                                                    Min<Prod<Prod<Prod<M1, M2>, M3>, Prod<M4, Prod<M5, M6>>>,
                                                                                        Min<Prod<Prod<Prod<M1, M2>, M3>, Prod<Prod<M4, M5>, M6>>,
                                                                                            Min<Prod<Prod<M1, Prod<M2, Prod<M3, M4>>>, Prod<M5, M6>>,
                                                                                                Min<Prod<Prod<M1, Prod<Prod<M2, M3>, M4>>, Prod<M5, M6>>,
                                                                                                    Min<Prod<Prod<Prod<M1, M2>, Prod<M3, M4>>, Prod<M5, M6>>,
                                                                                                        Min<Prod<Prod<Prod<M1, Prod<M2, M3>>, M4>, Prod<M5, M6>>,
                                                                                                            Min<Prod<Prod<Prod<Prod<M1, M2>, M3>, M4>, Prod<M5, M6>>,
                                                                                                                Min<Prod<Prod<M1, Prod<M2, Prod<M3, Prod<M4, M5>>>>, M6>,
                                                                                                                    Min<Prod<Prod<M1, Prod<M2, Prod<Prod<M3, M4>, M5>>>, M6>,
                                                                                                                        Min<Prod<Prod<M1, Prod<Prod<M2, M3>, Prod<M4, M5>>>, M6>,
                                                                                                                            Min<Prod<Prod<M1, Prod<Prod<M2, Prod<M3, M4>>, M5>>, M6>,
                                                                                                                                Min<Prod<Prod<M1, Prod<Prod<Prod<M2, M3>, M4>, M5>>, M6>,
                                                                                                                                    Min<Prod<Prod<Prod<M1, M2>, Prod<M3, Prod<M4, M5>>>, M6>,
                                                                                                                                        Min<Prod<Prod<Prod<M1, M2>, Prod<Prod<M3, M4>, M5>>, M6>,
                                                                                                                                            Min<Prod<Prod<Prod<M1, Prod<M2, M3>>, Prod<M4, M5>>, M6>,
                                                                                                                                                Min<Prod<Prod<Prod<Prod<M1, M2>, M3>, Prod<M4, M5>>, M6>,
                                                                                                                                                    Min<Prod<Prod<Prod<M1, Prod<M2, Prod<M3, M4>>>, M5>, M6>,
                                                                                                                                                        Min<Prod<Prod<Prod<M1, Prod<Prod<M2, M3>, M4>>, M5>, M6>,
                                                                                                                                                            Min<Prod<Prod<Prod<Prod<M1, M2>, Prod<M3, M4>>, M5>, M6>,
                                                                                                                                                                Min<Prod<Prod<Prod<Prod<M1, Prod<M2, M3>>, M4>, M5>, M6>,
                                                                                                                                                                    Prod<Prod<Prod<Prod<Prod<M1, M2>, M3>, M4>, M5>, M6>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>;
}