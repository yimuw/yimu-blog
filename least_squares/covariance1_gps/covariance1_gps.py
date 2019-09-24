import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple

SatelliteMeasurement = namedtuple(
    'SatelliteMeasurement', ['position', 'distant'])


def generate_data():
    receiver_position = np.array([-10, 0.])

    satelite_positions = [np.array([100, 100]), np.array(
        [100, -100]), np.array([-100, -200])]

    measurements = [SatelliteMeasurement(pos, distant=np.linalg.norm(receiver_position - pos))
                    for pos in satelite_positions]

    return measurements


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

            delta = self.solve_normal_equation(jacobian, b)

            self.variables_xy += delta
            print('position xy:', self.variables_xy)

    def h_function(self, satelite_measurement):
        dist = self.variables_xy - satelite_measurement.position
        hx = dist.T @ dist
        return hx

    def residaul(self, satelite_measurement):
        dist2 = satelite_measurement.distant * satelite_measurement.distant
        return self.h_function(satelite_measurement) - dist2

    def compute_cost(self, satelite_measurements):
        cost = 0.
        for s in satelite_measurements:
            r = self.residaul(s)
            cost += r * r
        cost /= len(satelite_measurements)
        return cost

    def compute_jacobian(self, satelite_measurements):
        num_residauls = len(satelite_measurements)
        jacobian = np.zeros([num_residauls, 2])

        for i, s in enumerate(satelite_measurements):
            jacobian[i, :] = 2 * (self.variables_xy - s.position)

        return jacobian

    def gradient_checking_simple(self, satelite_measurement):
        x_orig = self.variables_xy

        delta = 1e-3
        self.variables_xy = x_orig
        self.variables_xy += np.array([delta, 0])
        r_plus = self.residaul(satelite_measurement)
        
        self.variables_xy = x_orig
        self.variables_xy += np.array([-delta, 0])
        r_minus = self.residaul(satelite_measurement)
        grad_x = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = x_orig
        self.variables_xy += np.array([0, delta])
        r_plus = self.residaul(satelite_measurement)
        
        self.variables_xy = x_orig
        self.variables_xy += np.array([0, -delta])
        r_minus = self.residaul(satelite_measurement)
        grad_y = (r_plus - r_minus) / (2 * delta)

        self.variables_xy = x_orig

        return [grad_x, grad_y]

    def compute_b(self, satelite_measurements):
        num_residauls = len(satelite_measurements)
        b = np.zeros([num_residauls, 1])
        for i, s in enumerate(satelite_measurements):
            b[i, :] = self.residaul(s)

        return b

    def solve_normal_equation(self, jacobian, b):
        lhs = jacobian.T @ jacobian
        rhs = -jacobian.T @ b
        delta = np.linalg.solve(lhs, rhs)
        delta = delta.flatten()
        print('delta: ', delta)
        return delta


def main():
    satelite_measurements = generate_data()

    gps = GPS(satelite_measurements)
    gps.least_squares()


if __name__ == "__main__":
    main()
