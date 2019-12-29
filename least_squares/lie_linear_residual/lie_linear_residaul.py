import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


def random_angle():
    import random
    return random.uniform(-np.pi, np.pi)


def skew_symmetric(w):
    """
      The cross operator for vector3d
    """
    w1, w2, w3 = w
    return np.array([
        [0, -w3, w2],  # NOLINT
        [w3, 0, -w1],  # NOLINT
        [-w2, w1, 0]
    ])


def unskew_symmetric(W):
    assert np.allclose(W, -W.T)
    return np.array([W[2, 1], W[0, 2], W[1, 0]])


def skew_SO3_exp(w):
    from scipy.linalg import expm
    R = expm(skew_symmetric(w))
    assert np.allclose(R.T @ R, np.identity(3))
    return R


def SO3_log_unskew(R):
    from scipy.linalg import logm
    assert np.allclose(R.T @ R, np.identity(3))
    if np.allclose(R, R.T):
        # break sysmetry
        # e.g. turning from 0 to pi, you can do clock-wise or counter-close-wise.
        return SO3_log_unskew(R @ skew_SO3_exp([1e-6, 0, 0.]))
    return unskew_symmetric(logm(R))


class Problem:
    def __init__(self):
        self.R_target = skew_SO3_exp(
            [random_angle(), random_angle(),
             random_angle()])

    def residual_function(self, R):
        """
        A residual function r(R) = log(R_target.T @ R)
        """
        residual = SO3_log_unskew(self.R_target.T @ R)
        return residual

    def cost(self, R):
        """
        cost(R) = ||residual(R)||^2
        """
        r = self.residual_function(R)
        return r.T @ r

    def numerical_jacobian(self, R):
        """
        dr/dw = (r(w + dw) - r(w - dw)) / (2*dw)
        """
        # dr = jacobian @ dw
        jacobian = np.zeros([3, 3])
        DELTA = 1e-8
        for i in range(3):
            dw_plus = np.array([0., 0., 0.])
            dw_plus[i] = DELTA
            R_plus = R @ skew_SO3_exp(dw_plus)

            dw_minus = np.array([0., 0., 0.])
            dw_minus[i] = -DELTA
            R_minus = R @ skew_SO3_exp(dw_minus)

            j_i = (self.residual_function(R_plus) -
                   self.residual_function(R_minus)) / (2 * DELTA)
            jacobian[:, i] = j_i

        return jacobian


def gaussian_newton():
    R_variable = skew_SO3_exp([random_angle(), random_angle(), random_angle()])
    p = Problem()
    print("R_init:", R_variable)
    print('cost:', p.cost(R_variable))

    print('gaussian_newton...')
    residual = p.residual_function(R_variable)
    jacobian = p.numerical_jacobian(R_variable)
    dw = np.linalg.solve(jacobian.T @ jacobian, -jacobian.T @ residual)
    R_single_iteration = R_variable @ skew_SO3_exp(dw)

    print("R_single_iteration:", R_single_iteration)
    print("R_target:", p.R_target)
    assert np.allclose(R_single_iteration, p.R_target, 1e-4, 1e-6)

    print('single iter cost:', p.cost(R_single_iteration))


def plot_cost_sqrt():
    p = Problem()
    direction = np.random.rand(3, 1)
    direction = direction / np.linalg.norm(direction)

    deltas = np.linspace(-np.pi, 2 * np.pi, 300)
    costs = [p.cost(p.R_target @ skew_SO3_exp(d * direction)) for d in deltas]
    plt.subplot(2, 1, 1)
    plt.plot(deltas, costs)
    plt.title('change-along-a-direction vs cost')

    plt.subplot(2, 1, 2)
    plt.plot(deltas, [sqrt(cost) for cost in costs])
    plt.title('change-along-a-direction vs sqrt(cost)')
    plt.show()


def main():
    gaussian_newton()
    plot_cost_sqrt()


if __name__ == "__main__":
    main()