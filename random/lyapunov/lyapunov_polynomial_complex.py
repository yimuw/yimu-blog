# Import packages.
import cvxpy as cp
import numpy as np
import sympy
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
from matplotlib import cm


class LyapunovSOS:
    def __init__(self):
        # Warning: you need to copy constaints!
        x, y = sympy.symbols('x y')
        self.f = sympy.Matrix([
            [-y - 3/2*x**2 - 1/2*x**3],
            [3*x - y],
        ])

        self.b1 = sympy.Matrix(
            [[x, y, x**2, x*y, y**2, x**3, x**2*y, x*y**2, y**3]]).transpose()

    def polynomial_arrangement(self):
        x, y = sympy.symbols('x y')

        Q = sympy.MatrixSymbol('Q', 9, 9)

        dzdx = sympy.diff(self.b1, x)
        dzdy = sympy.diff(self.b1, y)

        B = sympy.Matrix([
            [dzdx, dzdy]
        ])

        V = (self.b1.T @ Q @ self.b1).as_explicit()
        V_poly = sympy.Poly(V[0], x, y)

        V_dot = (- 2 * self.b1.T @ Q @ B @ self.f).as_explicit()
        V_dot_poly = sympy.Poly(V_dot[0], x, y)

        b2 = sympy.Matrix(
            [[x, y, x**2, x*y, y**2, x**3, x**2*y, x*y**2, y**3]]).transpose()
        G = sympy.MatrixSymbol('G', 9, 9)
        SOS_of_V_dot = (b2.T @ G @ b2).as_explicit()
        SOS_of_V_dot_poly = sympy.Poly(SOS_of_V_dot[0], x, y)

        # Very tricky!
        b2_sqr = [b*b for b in b2]
        for b_sqr in b2_sqr:
            coeff = V_dot_poly.coeff_monomial(b_sqr)
            if coeff is sympy.S.Zero:
                print('WARNING: digonal coeff is zero for',
                      b_sqr, ' not PSD for sure!')

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
        num_var_q = 9
        Q = cp.Variable((num_var_q, num_var_q), symmetric=True)

        num_var_w = 9
        G = cp.Variable((num_var_w, num_var_w), symmetric=True)

        # sufficient condition
        Epsilon = 1e-8

        constraints = [Q >> Epsilon * np.identity(num_var_q)]
        constraints += [G >> Epsilon * np.identity(num_var_w)]

        constraints += [2*Q[1, 0] + 2*Q[1, 1] == G[1, 1], 2*Q[0, 0] + 2*Q[0, 1] - 6*Q[1, 1] == G[0, 1] + G[1, 0], -6*Q[0, 1] == G[0, 0], 2*Q[1, 3] + 4*Q[1, 4] + 2*Q[4, 0] + 2*Q[4, 1] == G[1, 4] + G[4, 1], 2*Q[0, 3] + 4*Q[0, 4] + 4*Q[1, 2] + 2*Q[1, 3] - 12*Q[1, 4] + 2*Q[3, 0] + 2*Q[3, 1] - 6*Q[4, 1] == G[0, 4] + G[1, 3] + G[3, 1] + G[4, 0], 4*Q[0, 2] + 2*Q[0, 3] - 12*Q[0, 4] + 3.0*Q[1, 0] - 6*Q[1, 3] + 2*Q[2, 0] + 2*Q[2, 1] - 6*Q[3, 1] == G[0, 3] + G[1, 2] + G[2, 1] + G[3, 0], 3.0*Q[0, 0] - 6*Q[0, 3] - 6*Q[2, 1] == G[0, 2] + G[2, 0], 2*Q[1, 7] + 6*Q[1, 8] + 2*Q[4, 3] + 4*Q[4, 4] + 2*Q[8, 0] + 2*Q[8, 1] == G[1, 8] + G[4, 4] + G[8, 1], 2*Q[0, 7] + 6*Q[0, 8] + 4*Q[1, 6] + 4*Q[1, 7] - 18*Q[1, 8] + 2*Q[3, 3] + 4*Q[3, 4] + 4*Q[4, 2] + 2*Q[4, 3] - 12*Q[4, 4] + 2*Q[7, 0] + 2*Q[7, 1] - 6*Q[8, 1] == G[0, 8] + G[1, 7] + G[3, 4] + G[4, 3] + G[7, 1] + G[8, 0], 4*Q[0, 6] + 4*Q[0, 7] - 18*Q[0, 8] + 3.0*Q[1, 3] + 6*Q[1, 5] + 2*Q[1, 6] - 12*Q[1, 7] + 2*Q[2, 3] + 4*Q[2, 4] + 4*Q[3, 2] + 2*Q[3, 3] - 12*Q[3, 4] + 3.0*Q[4, 0] - 6*Q[4, 3] + 2*Q[6, 0] + 2*Q[6, 1] - 6*Q[7, 1] == G[0, 7] + G[1, 6] + G[2, 4] + G[3, 3] + G[4, 2] + G[6, 1] + G[7, 0], 3.0*Q[0, 3] + 6*Q[0, 5] + 2*Q[0, 6] - 12*Q[0, 7] + 1.0*Q[1, 0] + 6.0*Q[1, 2] - 6*Q[1, 6] + 4*Q[2, 2] + 2*Q[2, 3] - 12*Q[2, 4] + 3.0*Q[3, 0] - 6*Q[3, 3] + 2*Q[5, 0] + 2*Q[5, 1] - 6*Q[6, 1] == G[0, 6] + G[1, 5] + G[2, 3] + G[3, 2] + G[5, 1] + G[6, 0], 1.0*Q[0, 0] + 6.0*Q[0, 2] - 6*Q[0, 6] + 3.0*Q[2, 0] - 6*Q[2, 3] - 6*Q[5, 1] == G[0, 5] + G[2, 2] + G[5, 0], 2*Q[4, 7] + 6*Q[4, 8] + 2*Q[8, 3] + 4*Q[8, 4] == G[4, 8] + G[8, 4], 2*Q[3, 7] + 6*Q[3, 8] + 4*Q[4, 6] + 4*Q[4, 7] - 18*Q[4, 8] + 2*Q[7, 3] + 4*Q[7, 4] + 4*Q[8, 2] + 2*Q[8, 3] - 12*Q[8, 4] == G[3, 8] + G[4, 7] + G[7, 4] + G[8, 3], 3.0*Q[1, 7] + 2*Q[2, 7] + 6*Q[2, 8] + 4*Q[3, 6] + 4*Q[3, 7] - 18*Q[3, 8] + 3.0*Q[4, 3] + 6*Q[4, 5] + 2*Q[4, 6] - 12*Q[4, 7] + 2*Q[6, 3] + 4*Q[6, 4] +
                        4*Q[7, 2] + 2*Q[7, 3] - 12*Q[7, 4] + 3.0*Q[8, 0] - 6*Q[8, 3] == G[2, 8] + G[3, 7] + G[4, 6] + G[6, 4] + G[7, 3] + G[8, 2], 3.0*Q[0, 7] + 1.0*Q[1, 3] + 6.0*Q[1, 6] + 4*Q[2, 6] + 4*Q[2, 7] - 18*Q[2, 8] + 3.0*Q[3, 3] + 6*Q[3, 5] + 2*Q[3, 6] - 12*Q[3, 7] + 1.0*Q[4, 0] + 6.0*Q[4, 2] - 6*Q[4, 6] + 2*Q[5, 3] + 4*Q[5, 4] + 4*Q[6, 2] + 2*Q[6, 3] - 12*Q[6, 4] + 3.0*Q[7, 0] - 6*Q[7, 3] == G[2, 7] + G[3, 6] + G[4, 5] + G[5, 4] + G[6, 3] + G[7, 2], 1.0*Q[0, 3] + 6.0*Q[0, 6] + 2.0*Q[1, 2] + 9.0*Q[1, 5] + 3.0*Q[2, 3] + 6*Q[2, 5] + 2*Q[2, 6] - 12*Q[2, 7] + 1.0*Q[3, 0] + 6.0*Q[3, 2] - 6*Q[3, 6] + 4*Q[5, 2] + 2*Q[5, 3] - 12*Q[5, 4] + 3.0*Q[6, 0] - 6*Q[6, 3] == G[2, 6] + G[3, 5] + G[5, 3] + G[6, 2], 2.0*Q[0, 2] + 9.0*Q[0, 5] + 1.0*Q[2, 0] + 6.0*Q[2, 2] - 6*Q[2, 6] + 3.0*Q[5, 0] - 6*Q[5, 3] == G[2, 5] + G[5, 2], 2*Q[8, 7] + 6*Q[8, 8] == G[8, 8], 2*Q[7, 7] + 6*Q[7, 8] + 4*Q[8, 6] + 4*Q[8, 7] - 18*Q[8, 8] == G[7, 8] + G[8, 7], 3.0*Q[4, 7] + 2*Q[6, 7] + 6*Q[6, 8] + 4*Q[7, 6] + 4*Q[7, 7] - 18*Q[7, 8] + 3.0*Q[8, 3] + 6*Q[8, 5] + 2*Q[8, 6] - 12*Q[8, 7] == G[6, 8] + G[7, 7] + G[8, 6], 1.0*Q[1, 7] + 3.0*Q[3, 7] + 1.0*Q[4, 3] + 6.0*Q[4, 6] + 2*Q[5, 7] + 6*Q[5, 8] + 4*Q[6, 6] + 4*Q[6, 7] - 18*Q[6, 8] + 3.0*Q[7, 3] + 6*Q[7, 5] + 2*Q[7, 6] - 12*Q[7, 7] + 1.0*Q[8, 0] + 6.0*Q[8, 2] - 6*Q[8, 6] == G[5, 8] + G[6, 7] + G[7, 6] + G[8, 5], 1.0*Q[0, 7] + 2.0*Q[1, 6] + 3.0*Q[2, 7] + 1.0*Q[3, 3] + 6.0*Q[3, 6] + 2.0*Q[4, 2] + 9.0*Q[4, 5] + 4*Q[5, 6] + 4*Q[5, 7] - 18*Q[5, 8] + 3.0*Q[6, 3] + 6*Q[6, 5] + 2*Q[6, 6] - 12*Q[6, 7] + 1.0*Q[7, 0] + 6.0*Q[7, 2] - 6*Q[7, 6] == G[5, 7] + G[6, 6] + G[7, 5], 2.0*Q[0, 6] + 3.0*Q[1, 5] + 1.0*Q[2, 3] + 6.0*Q[2, 6] + 2.0*Q[3, 2] + 9.0*Q[3, 5] + 3.0*Q[5, 3] + 6*Q[5, 5] + 2*Q[5, 6] - 12*Q[5, 7] + 1.0*Q[6, 0] + 6.0*Q[6, 2] - 6*Q[6, 6] == G[5, 6] + G[6, 5], 3.0*Q[0, 5] + 2.0*Q[2, 2] + 9.0*Q[2, 5] + 1.0*Q[5, 0] + 6.0*Q[5, 2] - 6*Q[5, 6] == G[5, 5]]

        # It is even possible to pick a particularly elegant solution, given by a quadratic Lyapunov function.
        # This can be achieved by minimizing the sum of diagonal elements corresponding to
        # the nonquadratic terms, subject to the LMI constraints.
        C = np.identity(num_var_q)
        C[0:2, 0:2] = 0.

        prob = cp.Problem(cp.Minimize(cp.trace(C @ Q)),
                          constraints)
        prob.solve(verbose=False)

        # Print result.
        print("status:", prob.status)
        print("The optimal value is", prob.value)
        print("A solution Q is")
        print(Q.value)

        V_optimal = self.b1.T @ Q.value @ self.b1

        x, y = sympy.symbols('x y')
        V_func = sympy.utilities.lambdify([x, y], V_optimal)

        self.plot(V_func)

    def plot(self, V_func):
        # Make data.
        X = np.arange(-5, 5, 0.1)
        Y = np.arange(-5, 5, 0.1)
        X, Y = np.meshgrid(X, Y)

        V = np.zeros_like(X)

        (len1, len2) = X.shape
        for y in range(len1):
            for x in range(len2):
                V[y, x] = V_func(X[y, x], Y[y, x])

        # Plot the surface in 3d.
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        surf = ax.plot_surface(X, Y, V, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        plt.title('Lyapunov function for engine')
        plt.show()

        # sparse
        X2 = np.arange(-5, 5, 0.5)
        Y2 = np.arange(-5, 5, 0.5)
        X2, Y2 = np.meshgrid(X2, Y2)
        DX = -Y2 - 3/2 * (X2 * X2) - 1/2 * (X2 * X2 * X2)
        DY = 3*X2 - Y2

        plt.quiver(X2, Y2, DX, DY)
        plt.contour(X, Y, V, colors='r', levels=[
                    1e-7, 5e-7, 1e-6, 5e-6, 1e-5, 5e-5, 1e-4, 5e-4])
        plt.axis('equal')
        plt.title('Lyapunov function for engine\n level-sets and dynamic-fields')

        plt.show()


def main():
    lyapunov = LyapunovSOS()
    lyapunov.polynomial_arrangement()
    lyapunov.solve_sos_as_sdp()


if __name__ == "__main__":
    main()
