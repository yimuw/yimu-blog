import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
from collections import namedtuple

SatelliteMeasurement = namedtuple(
    'SatelliteMeasurement', ['position', 'distant', 'weight'])


def generate_data():
    receiver_position = np.array([-10, 0.])

    satelite_positions = [np.array([100, 100]), np.array(
        [100, -100]), np.array([-100, -200])]

    satelite_positions = [np.array([0, 100]), np.array(
        [0, 80]), np.array([0, 70])]

    #satelite_positions = [np.array([0, 100]), np.array(
    #    [0, -100])]

    # satelite_positions = [np.array([100, 100])]

    distance_noisy_std = 0.1

    measurements = [SatelliteMeasurement(position=pos, 
                        distant=np.linalg.norm(receiver_position - pos 
                        + np.random.normal(0, distance_noisy_std, 1)),
                        weight=1)
                    for pos in satelite_positions]

    return measurements

# https://stackoverflow.com/questions/20126061/creating-a-confidence-ellipses-in-a-sccatterplot-using-matplotlib
def plot_hessian_as_covarinace_2d(xy, hessian, satelite_measurements):
    x, y = xy
    cov = np.linalg.inv(hessian)
    lambda_, v = np.linalg.eig(cov)
    lambda_ = np.sqrt(lambda_)
    ax = plt.subplot(111, aspect='equal')
    for j in range(1, 4):
        ell = Ellipse(xy=(np.mean(x), np.mean(y)),
                    width=lambda_[0]*j*2, height=lambda_[1]*j*2,
                    angle=np.rad2deg(np.arccos(v[0, 0])),
                    facecolor='none', edgecolor='red')
        ax.add_artist(ell)
    for s in satelite_measurements:
        plt.scatter(*s.position)
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    plt.scatter(x, y)
    plt.show()


class GPS:
    '''
    want:
        minimize_xy = sum_i ||h_i(xy) - dist_i||^2
        where h_i(xy) = dist(xy, satelite_i)
    '''

    def __init__(self, satelite_measurements):
        # states
        self.variables_xy = np.array([0, 0.])

        # data
        self.satelite_measurements = satelite_measurements

        # config
        self.max_iteration = 5

    def least_squares(self):

        for iteration in range(self.max_iteration):

            cost = self.compute_cost(self.satelite_measurements)
            print('cost:', cost)

            jacobian = self.compute_jacobian(self.satelite_measurements)

            b = self.compute_b(self.satelite_measurements)

            W = self.compute_weights(self.satelite_measurements)

            delta = self.solve_normal_equation(jacobian, b, W)

            self.variables_xy += delta
            print('position xy:', self.variables_xy)
    def h_function(self, satelite_measurement):
        diff = self.variables_xy - satelite_measurement.position
        dist = np.linalg.norm(diff)
        return dist

    def residaul(self, satelite_measurement):
        return self.h_function(satelite_measurement) - satelite_measurement.distant

    def compute_cost(self, satelite_measurements):
        cost = 0.
        for s in satelite_measurements:
            r = self.residaul(s)
            cost += r * s.weight * r
        cost /= len(satelite_measurements)
        return cost

    def compute_jacobian(self, satelite_measurements):
        num_residauls = len(satelite_measurements)
        jacobian = np.zeros([num_residauls, 2])

        for i, s in enumerate(satelite_measurements):
            f = self.variables_xy - s.position
            print(f)
            jacobian[i, :] = f / np.linalg.norm(f)
            print(jacobian)
            np.testing.assert_allclose(jacobian[i, :], 
                self.gradient_checking_simple(satelite_measurements[i]))

        return jacobian

    def compute_weights(self, satelite_measurements):
        num_residauls = len(satelite_measurements)
        W_diag = [s.weight for s in satelite_measurements]
        W = np.diag(W_diag)
        return W

    def gradient_checking_simple(self, satelite_measurement):
        x_orig = np.copy(self.variables_xy)

        delta = 1e-6
        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([delta, 0])
        r_plus = self.residaul(satelite_measurement)
        
        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([-delta, 0])
        r_minus = self.residaul(satelite_measurement)
        grad_x = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([0, delta])
        r_plus = self.residaul(satelite_measurement)
        
        self.variables_xy = np.copy(x_orig)
        self.variables_xy += np.array([0, -delta])
        r_minus = self.residaul(satelite_measurement)
        grad_y = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = np.copy(x_orig)

        return [grad_x, grad_y]

    def compute_b(self, satelite_measurements):
        num_residauls = len(satelite_measurements)
        b = np.zeros([num_residauls, 1])
        for i, s in enumerate(satelite_measurements):
            b[i, :] = self.residaul(s)

        return b

    def solve_normal_equation(self, jacobian, b, W):
        lhs = jacobian.T @ W @ jacobian
        rhs = -jacobian.T @ W @ b
        delta = np.linalg.solve(lhs, rhs)
        delta = delta.flatten()

        self.hessian = lhs

        print('delta: ', delta)
        print('hesian:', self.hessian)
        return delta

    def plot_cost(self):
        dx = dy = 0.5
        Y, X = np.mgrid[slice(-50, 50 + dy, dy),
                slice(-50, 50 + dx, dx)]

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
        plt.show()
        


def main():
    satelite_measurements = generate_data()

    gps = GPS(satelite_measurements)
    gps.plot_cost()
    gps.least_squares()

    plot_hessian_as_covarinace_2d(gps.variables_xy, gps.hessian, gps.satelite_measurements)
    plt.show()


if __name__ == "__main__":
    main()
