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

        q1, q2, q3, q4, q5, q6, q7, q8, q9, q10 = sympy.symbols('q1 q2 q3 q4 q5 q6 q7 q8 q9 q10')

        Q = sympy.Matrix([
            [q1, q2, q3, q4],
            [0, q5, q6, q7],
            [0,  0, q8, q9],
            [0,  0,  0, q10],
        ])

        A = np.array([[0, -0.5, -0.5, 0],
                      [-0.5, 0, 0, -0.5],
        ])

        z = sympy.Matrix([[x, x*x, y, y*y]]).transpose()

        B = sympy.Matrix([
            [1, 0],
            [2*x, 0],
            [0, 1],
            [0, 2*y]
        ])

        print(z)
        print(Q)
        print(B)

        V_dot = z.T @ Q @ B @ A @ z
        # V_dot_poly = sympy.simplify(V_dot[0])
        V_dot_poly = sympy.Poly(V_dot[0], x, y)


        w = sympy.Matrix([x, x**2, x**3, y, y**2, y**3])
        G = sympy.MatrixSymbol('G', 6, 6)
        V_dot_SOS = (w.T @ G @ w).as_explicit()
        V_dot_SOS_poly =  sympy.Poly(V_dot_SOS[0], x, y)
        print('V_dot_SOS_poly:', V_dot_SOS_poly)

        MAX_ORDER = 5
        for x_order in range(0, MAX_ORDER + 1):
            for y_order in range(0, MAX_ORDER + 1):
                if y_order > x_order and y_order > 0 and x_order > 0:
                    continue

                coeff = V_dot_poly.coeff_monomial(x**x_order * y ** y_order)
                if not coeff is sympy.S.Zero:

                    print('order: (x= {},y= {})'.format(x_order, y_order), ' coeff:', coeff)
                    coresponding_idx_in_G = V_dot_SOS_poly.coeff_monomial(x**x_order * y ** y_order)
                    print('   index in G:', coresponding_idx_in_G)


    def solve_sos_as_sdp(self):
        num_var_z = 4
        Q = cp.Variable((num_var_z, num_var_z), symmetric=True)

        # assuming w is [x, x^2, x^3, y, y^2, y^3]
        num_var_w = 6
        slack_V_dot = cp.Variable((num_var_w, num_var_w), symmetric=True)
        # Q.value = np.identity(num_var)

        # sufficient condition
        Epsilon = 1e-7

        constraints = [Q >> Epsilon * np.identity(num_var_z)]
        constraints += [slack_V_dot << Epsilon * np.identity(num_var_w)]

        q1 = Q[0,0]; q2 = Q[0,1]; q3 = Q[0,2]; q4 = Q[0,3]
        q5 = Q[1,1]; q6 = Q[1,2]; q7 = Q[1,3]
        q8 = Q[2,2]; q9 = Q[2,3]
        q10 = Q[3,3]

        constraints += [0 == slack_V_dot[5,3]]
        constraints += [0 == slack_V_dot[0,2]]

        constraints += [-0.5*q8 == slack_V_dot[4,3] * 2.]
        constraints += [-1.0*q9 == slack_V_dot[4,4] * 2.]
        constraints += [-1.0*q10 == slack_V_dot[5,4] * 2.]
        constraints += [-0.5*q1 - 0.5*q8 == slack_V_dot[3,0] * 2.]
        constraints += [-0.5*q3 == slack_V_dot[0,0] * 2.]
        constraints += [-1.0*q2 - 1.0*q4 == slack_V_dot[3,1] * 2.]
        constraints += [-0.5*q6 == slack_V_dot[4,1] * 2.]
        constraints += [-0.5*q1 - 0.5*q6 == slack_V_dot[1,0] * 2.]
        constraints += [-1.0*q5 - 1.0*q7 == slack_V_dot[3,2] * 2.]
        constraints += [-1.0*q2 == slack_V_dot[1,1] * 2.]
        constraints += [-1.0*q5 == slack_V_dot[2,1] * 2.]

        prob = cp.Problem(cp.Minimize(1),
                        constraints)
        prob.solve(verbose = False)

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