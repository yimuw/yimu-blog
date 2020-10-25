from number_forward_flow import *


def linear_case():
    print('=============== linear_case ==============')
    A = np.random.rand(6, 5)
    x_gt = np.ones(5, dtype='float64')

    b = A @ x_gt

    def residual_test(vars):
        """
        r = Ax - b
        """
        ret = A @ vars - b
        return ret

    x0 = np.random.rand(5, 1)
    r, J = ResidualBlock(residual_test, x0).evaluate()
    print('r:', r)
    print('J:', J)
    print('A:', A)

    J = np.array(J)
    r = np.array(r)
    dx = np.linalg.solve(J.T @ J, -J.T @ r)
    print('x0:', x0.T)
    print('dx:', dx.T)
    print('solver res:', (x0 + dx).T)
    print('x_gt:', x_gt.T)


if __name__ == "__main__":
    linear_case()
