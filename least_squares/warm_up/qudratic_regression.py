import numpy as np
import matplotlib.pyplot as plt


def generate_quadratic_data():
    quadratic_a = 2.4
    quadratic_b = -2.
    quadratic_c = 1.

    num_data = 30

    noise = 2 * np.random.randn(num_data)

    sampled_x = np.linspace(-10, 10., num_data)
    sampled_y = quadratic_a + sampled_x * sampled_x + quadratic_b * sampled_x + quadratic_c + noise

    return sampled_x, sampled_y


class LeastSquares:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.data_mat = np.vstack(
            [self.x * self.x, self.x,
             np.ones_like(self.x)]).T

        self.theta = np.array([0, 0, 0.])

    def residual(self):
        pred_y = self.data_mat @ self.theta
        r = pred_y - self.y
        return r

    def cost(self):
        r = self.residual()
        return r.T @ r

    def compute_jacobian(self):
        return self.data_mat

    def least_squares_solve(self):
        for i in range(10):
            print('iteration: {} cost: {}'.format(i, self.cost()))
            J = self.compute_jacobian()
            r = self.residual()

            delta = np.linalg.solve(J.T @ J, -J.T @ r)
            self.theta += delta

            if np.linalg.norm(delta) < 1e-8:
                print('converged iteration: {} cost: {}'.format(
                    i, self.cost()))
                break

        return self.theta


def main():
    x_data, y_data = generate_quadratic_data()

    solver = LeastSquares(x_data, y_data)

    theta = solver.least_squares_solve()

    x = np.linspace(-12, 12., 100)
    a, b, c = theta
    pred_y = a * x * x + b * x + c

    p1 = plt.plot(x_data, y_data, '*')
    p2 = plt.plot(x, pred_y, 'g')
    plt.legend((p1[0], p2[0]), ('sampled data', 'fitted function'))
    plt.title('Data points vs Fitted curve')
    plt.show()


if __name__ == "__main__":
    main()
