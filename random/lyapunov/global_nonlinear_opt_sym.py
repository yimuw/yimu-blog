# Import packages.
import cvxpy as cp
import numpy as np
import sympy


class GlobalPolynomialOptimization:
    def __init__(self):
        pass

    def coefficient_symbolic_match(self):
        # f(x, y) = 4 x^2 - 21/10* x^4 + 1/3 x^6 + xy - 4y^2 + 4y^4
        
        x, y, gamma = sympy.symbols('x y gamma')

        f_monomials = [x**2, x**4, x**6, x*y, y**2, y**4]
        f_coeffs = [4., -21/10., 1/3., 1., -4., 4.]

        w = sympy.Matrix([1, x, x**2, x**3, y, y**2, y**3, x*y, x*y*y, x*x*y])
        Q = sympy.MatrixSymbol('Q', 10, 10)
        V_dot_SOS = (w.T @ Q @ w).as_explicit()
        V_dot_SOS_poly = sympy.Poly(V_dot_SOS[0], x, y)
        print('V_dot_SOS_poly:', V_dot_SOS_poly)


        constraint_list_poly = []
        for f_monomial, f_coeff in zip(f_monomials, f_coeffs):
            Q_coeff = V_dot_SOS_poly.coeff_monomial(f_monomial)
            constrain = '{}=={}'.format(Q_coeff, f_coeff)
            print('constrain:', constrain)

            constraint_list_poly.append(constrain)
        print(','.join(constraint_list_poly))

        MAX_ORDER = 10
        constraint_list_zero = []
        for x_order in range(0, MAX_ORDER + 1):
            for y_order in range(0, MAX_ORDER + 1):
                # skip symmetry
                if y_order > x_order and y_order > 0 and x_order > 0:
                    continue
                # skip constant
                if y_order == 0 and x_order == 0:
                    continue

                monomial = x**x_order * y ** y_order
                # skip non-zero coef
                if monomial in f_monomials:
                    continue

                coeff = V_dot_SOS_poly.coeff_monomial(monomial)
                if not coeff is sympy.S.Zero:
                    constrain = '{} == 0'.format(coeff)
                    print('constrain:', constrain, 'for coef:', x**x_order * y ** y_order)
                    constraint_list_zero.append(constrain)
        print(','.join(constraint_list_zero))

    def solve_sos_as_sdp(self):
        num_var_w = 10
        Q = cp.Variable((num_var_w, num_var_w), symmetric=True)
        gamma = cp.Variable()

        # sufficient condition
        Epsilon = 1e-10

        constraints = [Q >> Epsilon * np.identity(num_var_w)]
        constraints += [Q[0, 0] == -gamma]
        
        constraints += [Q[0, 2] + Q[1, 1] + Q[2, 0]==4.0,Q[1, 3] + Q[2, 2] + Q[3, 1]==-2.1,Q[3, 3]==0.3333333333333333,Q[0, 7] + Q[1, 4] + Q[4, 1] + Q[7, 0]==1.0,Q[0, 5] + Q[4, 4] + Q[5, 0]==-4.0,Q[4, 6] + Q[5, 5] + Q[6, 4]==4.0]

        constraints += [Q[0, 4] + Q[4, 0] == 0,Q[0, 6] + Q[4, 5] + Q[5, 4] + Q[6, 0] == 0,Q[5, 6] + Q[6, 5] == 0,Q[6, 6] == 0,Q[0, 1] + Q[1, 0] == 0,Q[0, 9] + Q[1, 7] + Q[2, 4] + Q[4, 2] + Q[7, 1] + Q[9, 0] == 0,Q[1, 8] + Q[2, 5] + Q[4, 9] + Q[5, 2] + Q[7, 7] + Q[8, 1] + Q[9, 4] == 0,Q[0, 3] + Q[1, 2] + Q[2, 1] + Q[3, 0] == 0,Q[1, 9] + Q[2, 7] + Q[3, 4] + Q[4, 3] + Q[7, 2] + Q[9, 1] == 0,Q[2, 8] + Q[3, 5] + Q[5, 3] + Q[7, 9] + Q[8, 2] + Q[9, 7] == 0,Q[3, 6] + Q[6, 3] + Q[8, 9] + Q[9, 8] == 0,Q[2, 9] + Q[3, 7] + Q[7, 3] + Q[9, 2] == 0,Q[3, 8] + Q[8, 3] + Q[9, 9] == 0,Q[2, 3] + Q[3, 2] == 0,Q[3, 9] + Q[9, 3] == 0]

        prob = cp.Problem(cp.Minimize(-gamma),
                          constraints)
        prob.solve(verbose=False)

        # Print result.
        print("status:", prob.status)
        print("The optimal value is", prob.value)
        print("The low bound is", gamma.value)


def main():
    global_opt = GlobalPolynomialOptimization()
    global_opt.coefficient_symbolic_match()
    global_opt.solve_sos_as_sdp()


if __name__ == "__main__":
    main()
