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

        z = sympy.Matrix([[1, x, x**2, y, y**2, x*y]]).transpose()

        Q = sympy.MatrixSymbol('Q', 6, 6)
        
        # fx =  sympy.Matrix([
        #     [-y - 3/2*x**2 - 1/2*x**3],
        #     [3*x - y],
        # ])

        fx =  sympy.Matrix([
            [-0.5 * x],
            [-0.5 * y],
        ])

        dzdx = sympy.diff(z, x)
        dzdy = sympy.diff(z, y)

        B = sympy.Matrix([
            [dzdx, dzdy]
        ])

        print("B:", B)

        V = (z.T @ Q @ z).as_explicit()
        V_poly =  sympy.Poly(V[0], x, y)
        print('V_poly:', V_poly)

        V_dot = (- 2 * z.T @ Q @ B @ fx).as_explicit()
        V_dot_poly = sympy.Poly(V_dot[0], x, y)
        print('V_dot_poly:', V_dot_poly)

        w = sympy.Matrix([1, x, y, x**2, x*y, y**2, x**3, x**2*y, x*y**2, y**3])
        G = sympy.MatrixSymbol('G', 10, 10)
        SOS_of_V_dot = (w.T @ G @ w).as_explicit()
        SOS_of_V_dot_poly = sympy.Poly(SOS_of_V_dot[0], x, y)
        # print('SOS_of_V_dot_poly:', SOS_of_V_dot_poly)

        constraint_list = []
        for max_order in range(7):
            for x_order in range(0, max_order + 1):
                y_order = max_order - x_order
                # if y_order > x_order and y_order > 0 and x_order > 0:
                #     continue

                monomial = x ** x_order * y ** y_order

                coresponding_idx_in_G = SOS_of_V_dot_poly.coeff_monomial(
                    monomial)
                coeff = V_dot_poly.coeff_monomial(monomial)

                # print('monomial:', monomial)
                # print('monomial coeff:', coresponding_idx_in_G, coeff)

                if coeff is not sympy.S.Zero:
                    constrain = '{}=={}'.format(coeff, coresponding_idx_in_G)
                    print('constrain:', constrain, " of ", monomial)
                    constraint_list.append(constrain)
                else:
                    if coresponding_idx_in_G is not sympy.S.Zero:
                        constrain = '{}==0.'.format(coresponding_idx_in_G)
                        print('constrain:', constrain, " of ", monomial)
                        constraint_list.append(constrain)

        print(','.join(constraint_list))

    def solve_sos_as_sdp(self):
        num_var_q = 6
        Q = cp.Variable((num_var_q, num_var_q), symmetric=True)

        # assuming w is [1, x, x^2, x^3, y, y^2, y^3]
        num_var_w = 10
        G = cp.Variable((num_var_w, num_var_w), symmetric=True)
        # Q.value = np.identity(num_var)

        # sufficient condition
        Epsilon = 1e-4

        constraints = [Q >> Epsilon * np.identity(num_var_q)]
        constraints += [G >> Epsilon * np.identity(num_var_w)]

        constraints += [G[0, 0]==0.,2*Q[0, 1] + 2*Q[0, 3]==G[0, 2] + G[2, 0],-6*Q[0, 3]==G[0, 1] + G[1, 0],4*Q[0, 4] + 2*Q[0, 5] + 2*Q[3, 1] + 2*Q[3, 3]==G[0, 5] + G[2, 2] + G[5, 0],4*Q[0, 2] - 12*Q[0, 4] + 2*Q[0, 5] + 2*Q[1, 1] + 2*Q[1, 3] - 6*Q[3, 3]==G[0, 4] + G[1, 2] + G[2, 1] + G[4, 0],3.0*Q[0, 1] - 6*Q[0, 5] - 6*Q[1, 3]==G[0, 3] + G[1, 1] + G[3, 0],4*Q[3, 4] + 2*Q[3, 5] + 2*Q[4, 1] + 2*Q[4, 3]==G[0, 9] + G[2, 5] + G[5, 2] + G[9, 0],4*Q[1, 4] + 2*Q[1, 5] + 4*Q[3, 2] - 12*Q[3, 4] + 2*Q[3, 5] - 6*Q[4, 3] + 2*Q[5, 1] + 2*Q[5, 3]==G[0, 8] + G[1, 5] + G[2, 4] + G[4, 2] + G[5, 1] + G[8, 0],3.0*Q[0, 5] + 4*Q[1, 2] - 12*Q[1, 4] + 2*Q[1, 5] + 2*Q[2, 1] + 2*Q[2, 3] + 3.0*Q[3, 1] - 6*Q[3, 5] - 6*Q[5, 3]==G[0, 7] + G[1, 4] + G[2, 3] + G[3, 2] + G[4, 1] + G[7, 0],1.0*Q[0, 1] + 6.0*Q[0, 2] + 3.0*Q[1, 1] - 6*Q[1, 5] - 6*Q[2, 3]==G[0, 6] + G[1, 3] + G[3, 1] + G[6, 0],4*Q[4, 4] + 2*Q[4, 5]==G[2, 9] + G[5, 5] + G[9, 2],4*Q[4, 2] - 12*Q[4, 4] + 2*Q[4, 5] + 4*Q[5, 4] + 2*Q[5, 5]==G[1, 9] + G[2, 8] + G[4, 5] + G[5, 4] + G[8, 2] + G[9, 1],4*Q[2, 4] + 2*Q[2, 5] + 3.0*Q[3, 5] + 3.0*Q[4, 1] - 6*Q[4, 5] + 4*Q[5, 2] - 12*Q[5, 4] + 2*Q[5, 5]==G[1, 8] + G[2, 7] + G[3, 5] + G[4, 4] + G[5, 3] + G[7, 2] + G[8, 1],1.0*Q[0, 5] + 3.0*Q[1, 5] + 4*Q[2, 2] - 12*Q[2, 4] + 2*Q[2, 5] + 1.0*Q[3, 1] + 6.0*Q[3, 2] + 3.0*Q[5, 1] - 6*Q[5, 5]==G[1, 7] + G[2, 6] + G[3, 4] + G[4, 3] + G[6, 2] + G[7, 1],2.0*Q[0, 2] + 1.0*Q[1, 1] + 6.0*Q[1, 2] + 3.0*Q[2, 1] - 6*Q[2, 5]==G[1, 6] + G[3, 3] + G[6, 1],G[5, 9] + G[9, 5]==0.,G[4, 9] + G[5, 8] + G[8, 5] + G[9, 4]==0.,3.0*Q[4, 5]==G[3, 9] + G[4, 8] + G[5, 7] + G[7, 5] + G[8, 4] + G[9, 3],1.0*Q[3, 5] + 1.0*Q[4, 1] + 6.0*Q[4, 2] + 3.0*Q[5, 5]==G[3, 8] + G[4, 7] + G[5, 6] + G[6, 5] + G[7, 4] + G[8, 3],1.0*Q[1, 5] + 3.0*Q[2, 5] + 2.0*Q[3, 2] + 1.0*Q[5, 1] + 6.0*Q[5, 2]==G[3, 7] + G[4, 6] + G[6, 4] + G[7, 3],2.0*Q[1, 2] + 1.0*Q[2, 1] + 6.0*Q[2, 2]==G[3, 6] + G[6, 3],G[9, 9]==0.,G[8, 9] + G[9, 8]==0.,G[7, 9] + G[8, 8] + G[9, 7]==0.,1.0*Q[4, 5]==G[6, 9] + G[7, 8] + G[8, 7] + G[9, 6],2.0*Q[4, 2] + 1.0*Q[5, 5]==G[6, 8] + G[7, 7] + G[8, 6],1.0*Q[2, 5] + 2.0*Q[5, 2]==G[6, 7] + G[7, 6],2.0*Q[2, 2]==G[6, 6]]
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
