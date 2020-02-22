# Import packages.
import cvxpy as cp
import numpy as np

# linear case
# given a polynomial dynamic system dx = -y - 3/2 x^2 - 1/2 x^3
#                                   dy = 3x - y
# Find a quadratic Lyapunov function of z, V(z) = z^T Q z
# where z is polynomial basis of (x,y). z = [x, x^2, x^3, y, y^2, y^3]
#
# We can express [dx, dy] in the polynomial basis
#    dxy = A z
#
# want: V(z) > 0 => Q >> 0
#       dot V(x) < 0 => 
#       dot V(x) = dVdz * dzdx * dxdt
#    dVdz = 2 z^T Q
#    dzdx = [1, 2x, 3x^2, 0, 0 , 0   ]
#         = [0, 0 ,    0, 1, 2y, 3y^2]
#    dxdt is the d
# 2 z^T Q A z < 0 =>  - Q A - A^T Q > 0

class LinearSystemLyapunov:
    def __init__(self):
        pass

    def solve_for_quadratic_lyapunov(self, A):
        assert A.ndim == 2

        num_var = A.shape[1]

        Q = cp.Variable((num_var, num_var), symmetric=True)
        # Q.value = np.identity(num_var)

        # sufficient condition
        Epsilon = 1e-4 * np.identity(num_var)

        constraints = [Q >> Epsilon]
        constraints += [-Q @ A - A.T @ Q >> Epsilon]
        prob = cp.Problem(cp.Minimize(1),
                        constraints)
        prob.solve(verbose = False)

        # Print result.
        print("test the stability of linear system A:\n", A)
        print("status:", prob.status)
        print("The optimal value is", prob.value)
        print("A solution Q is")
        print(Q.value)


def main():
    # given a polynomial dynamic system dx = -y - 3/2 x^2 - 1/2 x^3
    #                                   dy = 3x - y
    # Find a quadratic Lyapunov function of z, V(z) = z^T Q z
    # where z is polynomial basis of (x,y). z = [x, x^2, x^3, y, y^2, y^3]

    A1 = np.array([
        [0, -3/2, -1/2, -1, 0, 0],
        [0, -0.1, 0, 0, 0, 0],
        [0, 0, -0.1, 0, 0, 0],
        [3, 0   , 0, -1, 0, 0],
        [0, 0, 0, 0, -0.1, 0],
        [0, 0, 0, 0, 0, -0.1],
    ])

    lyapunov = LinearSystemLyapunov()
    lyapunov.solve_for_quadratic_lyapunov(A1)



if __name__ == "__main__":
    main()