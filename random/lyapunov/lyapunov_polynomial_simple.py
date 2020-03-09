# Import packages.
import cvxpy as cp
import numpy as np
import sympy

# linear case
# given a polynomial dynamic system dx = - 0.5 y
#                                   dy = - 0.5 x
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
#           [0, 0 , 1, 2y]
#    dxdt is the fx
# 2 z^T Q B A z < 0 => - z^T Q B A z is S.O.S


class LinearSystemLyapunov:
    def __init__(self):
        pass

    def polynomial_arrangement(self):
        x, y = sympy.symbols('x y')

        z = sympy.Matrix([[x, y]]).transpose()

        Q = sympy.MatrixSymbol('Q', 2, 2)
        
        # fx =  sympy.Matrix([
        #     [-y - 3/2*x**2 - 1/2*x**3],
        #     [3*x - y],
        # ])

        fx =  sympy.Matrix([
            [-0.9 * x],
            [-0.5 * y],
        ])


        V = (z.T @ Q @ z).as_explicit()
        V_poly =  sympy.Poly(V[0], x, y)
        print('V_poly:', V_poly)

        V_dot = (- 2 * z.T @ Q @ fx).as_explicit()
        V_dot_poly = sympy.Poly(V_dot[0], x, y)
        print('V_dot_poly:', V_dot_poly)

        w = sympy.Matrix([x, y])
        G = sympy.MatrixSymbol('G', 2, 2)
        SOS_of_V_dot = (w.T @ G @ w).as_explicit()
        SOS_of_V_dot_poly = sympy.Poly(SOS_of_V_dot[0], x, y)
        # print('SOS_of_V_dot_poly:', SOS_of_V_dot_poly)

        constraint_list = []
        for max_order in range(10):
            for x_order in range(0, max_order + 1):
                y_order = max_order - x_order

                monomial = x ** x_order * y ** y_order

                SOS_coeff = SOS_of_V_dot_poly.coeff_monomial(
                    monomial)
                
                if SOS_coeff is sympy.S.Zero:
                    continue

                V_dot_coeff = V_dot_poly.coeff_monomial(monomial)

                if V_dot_coeff is not sympy.S.Zero:
                    constrain = '{}=={}'.format(V_dot_coeff, SOS_coeff)
                    print('constrain:', constrain, " of ", monomial)
                    constraint_list.append(constrain)
                else:
                    constrain = '{}==0.'.format(SOS_coeff)
                    print('constrain:', constrain, " of ", monomial)
                    constraint_list.append(constrain)

        print('Constraints (copy this!):', ','.join(constraint_list))

    def solve_sos_as_sdp(self):
        num_var_q = 2
        Q = cp.Variable((num_var_q, num_var_q), symmetric=True)

        # assuming w is [1, x, x^2, x^3, y, y^2, y^3]
        num_var_w = 2
        G = cp.Variable((num_var_w, num_var_w), symmetric=True)
        # Q.value = np.identity(num_var)

        # sufficient condition
        Epsilon = 1e-4

        constraints = [Q >> Epsilon * np.identity(num_var_q)]
        constraints += [G >> Epsilon * np.identity(num_var_w)]

        constraints += [1.0*Q[1, 1]==G[1, 1],1.0*Q[0, 1] + 1.8*Q[1, 0]==G[0, 1] + G[1, 0],1.8*Q[0, 0]==G[0, 0]]

        prob = cp.Problem(cp.Maximize(1),
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
