# Import packages.
import cvxpy as cp
import numpy as np
import sympy

# linear case
# given a polynomial dynamic system dx = - 0.5 y - 0.5 x^2
#                                   dy = - 0.5 x - 0.5 y^2
# Find a quadratic Lyapunov function of z, V(z) = z^T Q z
# where z is polynomial basis of (x,y). z = [x, x^2, y, y^2]
#
# We can express [dx, dy] in the polynomial basis
#    dxy = A z
#
# want: V(z) > 0 => Q >> 0
#
#       dot V(x) < 0 =>
#       dot V(x) = dVdz * dzdx * dxdt
#    dVdz = 2 z^T Q
#    dzdx = [1, 2x, 0, 0]  ^T = B
#         = [0, 0 , 1, 2y]
#    dxdt is the A
# 2 z^T Q B A z < 0 => - z^T Q B A z is S.O.S


class LinearSystemLyapunov:
    def __init__(self):
        pass

    def polynomial_arrangement(self):
        x, y = sympy.symbols('x y')

        e0, e1, e2, e3, e4, e5,e6, e7, e8, e9, e10, e11, e12, e13 = sympy.symbols(
            'e0 e1 e2 e3 e4 e5 e6 e7 e8 e9 e10 e11 e12 e13')

        V = e0 \
            + e1 * x + e2 * y \
            + e3 * x*x + e4 * y*y + e5 * x*y \
            + e6 * x**3 + e7 * x**2*y + e8 * x*y**2 + e9 * y**3 \
            + e10 * x**4 + e11 * x**3*y + e11 * x**2*y**2 + e12 * x*y**3 + e13 * y**4
        V = sympy.Poly(V, x, y)

        b1 = sympy.Matrix([[1, x, x**2, y, y**2, x*y]]).transpose()
        Q1 = sympy.MatrixSymbol('Q1', 6, 6)
        V_SOS = (b1.T @ Q1 @ b1).as_explicit()
        V_SOS_poly = sympy.Poly(V_SOS[0], x, y)

        constraint_list_Q1 = []
        for max_order in range(5): 
            for x_order in range(0, max_order + 1):
                y_order = max_order - x_order
                monomial = x**x_order * y ** y_order

                v_coeff = V.coeff_monomial(monomial)
                if not v_coeff is sympy.S.Zero:
                    coresponding_idx_in_Q1 = V_SOS_poly.coeff_monomial(
                        monomial)
                    constrain = '{}=={}'.format(v_coeff, coresponding_idx_in_Q1)
                    print('constrain:', constrain, ' monomial:', monomial)

                    constraint_list_Q1.append(constrain)
        print(','.join(constraint_list_Q1))



    def solve_sos_as_sdp(self):
        num_var_z = 4
        Q = cp.Variable((num_var_z, num_var_z), symmetric=True)

        # assuming w is [x, x^2, x^3, y, y^2, y^3]
        num_var_w = 6
        slack_V_dot = cp.Variable((num_var_w, num_var_w), symmetric=True)
        G = slack_V_dot
        # Q.value = np.identity(num_var)

        # sufficient condition
        Epsilon = 1

        constraints = [Q >> Epsilon * np.identity(num_var_z)]
        constraints += [slack_V_dot << Epsilon * np.identity(num_var_w)]

        q1 = Q[0, 0]
        q2 = Q[0, 1]
        q3 = Q[0, 2]
        q4 = Q[0, 3]
        q5 = Q[1, 1]
        q6 = Q[1, 2]
        q7 = Q[1, 3]
        q8 = Q[2, 2]
        q9 = Q[2, 3]
        q10 = Q[3, 3]

        constraints += [-0.5*q8 == G[3, 4] + G[4, 3], -2.0*q9 == G[3, 5] + G[4, 4] + G[5, 3], -1.0*q10 == G[4, 5] + G[5, 4], -0.5*q1 - 0.5*q8 == G[0, 3] + G[3, 0], -1.0*q3 == G[0, 0], -2.0*q2 - 2.0*q4 ==
                        G[1, 3] + G[3, 1], -1.0*q6 == G[1, 4] + G[4, 1], -0.5*q1 - 1.0*q6 == G[0, 1] + G[1, 0], -1.0*q5 - 2.0*q7 == G[2, 3] + G[3, 2], -2.0*q2 == G[0, 2] + G[1, 1] + G[2, 0], -1.0*q5 == G[1, 2] + G[2, 1]]

        prob = cp.Problem(cp.Minimize(1),
                          constraints)
        prob.solve(verbose=False)

        # Print result.
        print("status:", prob.status)
        print("The optimal value is", prob.value)
        print("A solution Q is")
        print(Q.value)


def main():
    lyapunov = LinearSystemLyapunov()
    lyapunov.polynomial_arrangement()
    lyapunov.solve_sos_as_sdp()


if __name__ == "__main__":
    main()
