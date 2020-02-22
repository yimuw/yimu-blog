# Import packages.
import cvxpy as cp
import numpy as np

# linear case
# given a linear dynamic system f(x) = Ax
# Find a quadratic Lyapunov function V(x) = x^T Q x
# want: V(x) > 0 => Q >> 0
#       dot V(x) < 0 => 2x^T Q A x < 0 =>  - Q A - A Q > 0

class LinearSystemLyapunov:
    def __init__(self):
        pass

    def solve_for_quadratic_lyapunov(self, A):
        assert A.ndim == 2
        assert A.shape[0] == A.shape[1]

        num_var = A.shape[0]

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
    A1 = np.array([
        [-0.5, 0, 0],
        [0, -0.5, 0],
        [0, 0, -0.5]
    ])

    lyapunov = LinearSystemLyapunov()
    lyapunov.solve_for_quadratic_lyapunov(A1)

    A2 = np.array([
        [2, 0, 0],
        [0, 2, 0],
        [0, 0, 2]
    ])
    lyapunov.solve_for_quadratic_lyapunov(A2)


if __name__ == "__main__":
    main()