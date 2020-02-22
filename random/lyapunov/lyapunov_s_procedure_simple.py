# Import packages.
import cvxpy as cp
import numpy as np

# verify p(x) >= 0 on g(x) > 0
# p(x) = (x-3)^2 - 1
# g(x) = 1 - x^2
#
# L = p(x) - l(x)g(x)
# l(x) := 1^T Q 1, sum of squares of 1
def solve_simple_constraint_sos():
    num_var = 1
   
    Q = cp.Variable((num_var, num_var), symmetric=True)

    slack = cp.Variable((2, 2), symmetric=True)
    # Q.value = np.identity(num_var)

    # sufficient condition
    Epsilon = 1e-4 * np.identity(num_var)

    constraints = [Q >> Epsilon]
    constraints += [slack >> Epsilon]

    constraints += [-Q + 8 == slack[0,0]]
    constraints += [-3 == slack[1,0]]
    constraints += [1+Q == slack[1,1]]

    prob = cp.Problem(cp.Minimize(1),
                    constraints)
    prob.solve(verbose = False)

    # Print result.
    print("status:", prob.status)
    print("The optimal value is", prob.value)
    print("A solution Q is")
    print(Q.value)


def main():
    solve_simple_constraint_sos()



if __name__ == "__main__":
    main()