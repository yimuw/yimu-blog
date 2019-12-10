import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
from collections import namedtuple

SatelliteMeasurement = namedtuple('SatelliteMeasurement',
                                  ['position', 'distant', 'weight'])


# Change satellite position here!
def generate_data():
    """
        Generate simulation data
    """
    receiver_position = np.array([-10, 0.])

    satelite_positions = [
        np.array([20, 20]),
        np.array([30, -30]),
        np.array([-40, 0]),
    ]

    # satelite_positions = [
    #     np.array([-2, 30]),
    #     np.array([2, 30]),
    # ]

    distance_noisy_std = 0.1

    measurements = [
        SatelliteMeasurement(
            position=pos,
            distant=np.linalg.norm(receiver_position - pos +
                                   np.random.normal(0, distance_noisy_std, 1)),
            weight=1) for pos in satelite_positions
    ]

    return measurements


def plot_hessian_as_covarinace_2d(ax, xy, hessian, satelite_measurements):
    """
        plot 2d hessian as cov

        https://stackoverflow.com/questions/20126061/creating-a-confidence-ellipses-in-a-sccatterplot-using-matplotlib
          you made a mistake, 
            angle=np.rad2deg(np.arctan2(v[0, 1], v[-1, 0]))
          not 
            angle=np.rad2deg(np.arccos(v[0, 0])))
    """
    x, y = xy
    cov = np.linalg.inv(hessian)
    value, v = np.linalg.eig(cov)
    value = np.sqrt(value)
    for j in range(1, 4):
        SCALE = 3
        ell = Ellipse(xy=(np.mean(x), np.mean(y)),
                      width=value[0] * j * SCALE,
                      height=value[1] * j * SCALE,
                      angle=np.rad2deg(np.arctan2(v[0, 1], v[1, 0])),
                      facecolor='none',
                      edgecolor='red')
        ax.add_artist(ell)

    info_string = 'satellite position: '
    for s in satelite_measurements:
        plt.scatter(*s.position)
        info_string += str(s.position) + ', '
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    plt.title(info_string)
    plt.scatter(x, y)


class GPS:
    """
    want:
        minimize_xy = sum_i ||h_i(xy) - dist_i||^2
        where h_i(xy) = dist(xy, satelite_i)
    """
    def __init__(self, satelite_measurements):
        # states
        self.variables_xy = np.array([0., 0.])

        # data
        self.satelite_measurements = satelite_measurements

        # config
        self.max_iteration = 5

    def least_squares(self):
        """
            The nonlinear least squares iteration
        """
        for iteration in range(self.max_iteration):
            cost = self.compute_cost(self.satelite_measurements)
            jacobian = self.compute_jacobian(self.satelite_measurements)
            b = self.compute_b(self.satelite_measurements)
            W = self.compute_weights(self.satelite_measurements)

            delta = self.solve_normal_equation(jacobian, b, W)

            self.variables_xy += delta
            print('cost:', cost, ' position xy:', self.variables_xy)

    def h_function(self, satelite_measurement):
        """
            The distance observation function
        """
        diff = self.variables_xy - satelite_measurement.position
        dist = np.linalg.norm(diff)
        return dist

    def residual(self, satelite_measurement):
        """
            The residual function for GPS
        """
        return self.h_function(
            satelite_measurement) - satelite_measurement.distant

    def compute_cost(self, satelite_measurements):
        """
            The cost function for GPS
        """
        cost = 0.
        for s in satelite_measurements:
            r = self.residual(s)
            cost += r * s.weight * r
        cost /= len(satelite_measurements)
        return cost

    def compute_jacobian(self, satelite_measurements):
        """
            Compute jacobian of residual function analytically
        """
        num_residuals = len(satelite_measurements)
        jacobian = np.zeros([num_residuals, 2])

        for i, s in enumerate(satelite_measurements):
            f = self.variables_xy - s.position
            jacobian[i, :] = f / np.linalg.norm(f)
            np.testing.assert_allclose(
                jacobian[i, :],
                self.gradient_checking_simple(satelite_measurements[i]), 1e-4)

        return jacobian

    def compute_weights(self, satelite_measurements):
        """
            Format the weight into a block-diagonal matrix
        """
        W_diag = [s.weight for s in satelite_measurements]
        W = np.diag(W_diag)
        return W

    def gradient_checking_simple(self, satelite_measurement):
        """
            Gradient checking
        """
        x_orig = np.copy(self.variables_xy)

        delta = 1e-6
        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([delta, 0])
        r_plus = self.residual(satelite_measurement)

        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([-delta, 0])
        r_minus = self.residual(satelite_measurement)
        grad_x = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([0, delta])
        r_plus = self.residual(satelite_measurement)

        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([0, -delta])
        r_minus = self.residual(satelite_measurement)
        grad_y = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = np.copy(x_orig)

        return [grad_x, grad_y]

    def compute_b(self, satelite_measurements):
        """
            residual function evaluated at current variables
        """
        num_residuals = len(satelite_measurements)
        b = np.zeros([num_residuals, 1])
        for i, s in enumerate(satelite_measurements):
            b[i, :] = self.residual(s)

        return b

    def solve_normal_equation(self, jacobian, b, W):
        """
            J^T J x = J^T b
        """
        lhs = jacobian.T @ W @ jacobian
        rhs = -jacobian.T @ W @ b
        delta = np.linalg.solve(lhs, rhs)
        delta = delta.flatten()

        self.hessian = lhs
        return delta

    def plot_cost(self):
        """
            Plot the cost field
        """
        dx = dy = 0.5
        Y, X = np.mgrid[slice(-50, 50 + dy, dy), slice(-50, 50 + dx, dx)]

        costs = np.zeros_like(X)

        cols, rows = X.shape
        for col in range(cols):
            for row in range(rows):
                x = X[col, row]
                y = Y[col, row]
                self.variables_xy = np.array([x, y])
                costs[col, row] = self.compute_cost(self.satelite_measurements)

        im = plt.pcolormesh(X, Y, costs)
        plt.colorbar(im)
        plt.title('cost field')
        plt.show()


def main():
    satelite_measurements = generate_data()

    gps = GPS(satelite_measurements)

    gps.least_squares()

    ax = plt.subplot(1, 2, 1, aspect='equal')
    plot_hessian_as_covarinace_2d(ax, gps.variables_xy, gps.hessian,
                                  gps.satelite_measurements)

    plt.subplot(1, 2, 2, aspect='equal')
    gps.plot_cost()

    plt.show()


if __name__ == "__main__":
    main()
