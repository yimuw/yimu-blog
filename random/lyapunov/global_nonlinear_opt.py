# Import packages.
import cvxpy as cp
import numpy as np

# f(x, y) = 4 x^2 - 21/10* x^4 + 1/3 x^6 + xy - 4y^2 + 4y^4
# want: min f(x, y)
#
# instead for s, check f(x) - s is S.O.S (> 0). So s is a lower bound for min f(x)
#
def solve_simple_constraint_sos():   
    num_ploy = 6
    A = cp.Variable((num_ploy, num_ploy), symmetric=True)
    low_bound = cp.Variable()
    # Q.value = np.identity(num_var)

    # sufficient condition
    Epsilon = 1e-4 * np.identity(num_ploy)

    constraints = [A >> Epsilon] 

    setted_index = set()

    a = 4; b = -21/10.; c = 1/3.; d= 1; e=-4; f=4

    # 1 * 1
    constraints+= [A[0, 0] == -low_bound]
    setted_index.add((0, 0))
    # x**2
    constraints+= [2 * A[0, 2] + A[1, 1] == a]
    setted_index.add((0, 2)); setted_index.add((1, 1))
    # x**3
    constraints+= [A[1, 2] + A[0, 3] == 0]
    setted_index.add((1, 2)); setted_index.add((0, 3))
    # x**4
    constraints+= [A[2, 2] + 2* A[1, 3] == b]
    setted_index.add((2, 2)); setted_index.add((1, 3))
    # x**6
    constraints+= [A[3, 3] == c]
    setted_index.add((3, 3))
    # y**2
    constraints+= [A[4, 4] + 2* A[0,5] == e]
    setted_index.add((4, 4)); setted_index.add((0, 5)) 
    # y ** 4
    constraints+= [A[5, 5] == f]
    setted_index.add((5, 5))
    # xy
    constraints+= [2 * A[1, 4] == d]
    setted_index.add((1, 4))

    for col in range(0, num_ploy):
        for row in range(0, col + 1):
            if not (row, col) in setted_index:
                print('row, col:', row, col)
                constraints+= [A[row, col] == 0.]

 
    prob = cp.Problem(cp.Minimize(-low_bound),
                    constraints)
    prob.solve(verbose = False)

    # Print result.
    print("status:", prob.status)
    print("The optimal value is", prob.value)
    print("The lower bound:", low_bound.value)


def main():
    solve_simple_constraint_sos()



if __name__ == "__main__":
    main()