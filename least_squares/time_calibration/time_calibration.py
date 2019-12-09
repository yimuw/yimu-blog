import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple

Function = namedtuple('Function', ['time', 'values'])


def generate_data():
    time = np.linspace(0., 20., 1000)
    values = np.cos(time) * time

    time_offset = 1.54321

    func1 = Function(time=time + time_offset, values=values)
    func2 = Function(time=time, values=values)

    if False:
        plt.plot(func1.time, func1.values)
        plt.plot(func2.time, func2.values)
        plt.title('function. ground truth')
        plt.show()

    return func1, func2, time_offset


class TimeCalibration:
    '''
    want:
        minimize_dt = sum_t ||func1(t + dt) - func2(t)||^2
    '''

    def __init__(self):
        # states
        self.variable_time = 0.

        # config
        self.max_iteration = 7

    def least_squares_calibrate(self, func1, func2):
        """
            It is the least square loop
        """
        for iteration in range(self.max_iteration):
            self.plot(func1, func2, 'fun1 vs fun2 at iter:{}, dt:{:.5f}'.format(
                iteration, self.variable_time))

            cost = self.compute_cost(func1, func2)
            print('iteration:{}  cost:{}'.format(iteration, cost))

            jacobian = self.compute_jacobian(func1)

            b = self.compute_b(func1, func2)

            delta = self.solve_normal_equation(jacobian, b)

            self.variable_time += delta

    def compute_cost(self, func1, func2):
        t = func1.time
        diff = np.interp(t + self.variable_time, func1.time, func1.values) \
            - np.interp(t, func2.time, func2.values)
        return diff.T @ diff

    def compute_jacobian(self, func1):
        """
            Compute the derivative of residual w.r.t dt
        """
        # compute derivative
        dt = func1.time[1] - func1.time[0]
        # np.convolve 'same' has boundary effect.
        value_padded = np.concatenate(
            [[func1.values[0]], func1.values, [func1.values[-1]]])
        dfunc1_dt = Function(func1.time, np.convolve(
            value_padded, [0.5 / dt, 0, - 0.5 / dt], 'valid'))

        SHOW_DEVRIVATIVE = False
        if SHOW_DEVRIVATIVE:
            plt.plot(dfunc1_dt.time, dfunc1_dt.values, 'r')
            plt.plot(func1.time, func1.values, 'g')
            plt.show()

        # r(dt) = func1(t+dt) - func2(t)
        # drdt(dt) = dfun1_dt(t + dt)
        jacobian = np.interp(func1.time + self.variable_time,
                             dfunc1_dt.time, dfunc1_dt.values)

        return jacobian

    def compute_b(self, func1, func2):
        """
            compute residual evaluated at current variable
        """
        # r(dt) = func1(t+dt) - func2(t)
        t = func1.time
        b = np.interp(t + self.variable_time, func1.time, func1.values) \
            - np.interp(t, func2.time, func2.values)
        return b

    def solve_normal_equation(self, jacobian, b):
        """
            solve the normal equation
        """
        lhs = jacobian.T @ jacobian
        rhs = -jacobian.T @ b

        # NOTE: np.linalg.solve(rhs, lhs) doesn't work for 1d
        delta = rhs / lhs
        return delta

    def plot(self, func1, func2, title='func1 vs func2'):
        """
            Plot func1 and func2
        """
        t = func1.time
        v1 = np.interp(t + self.variable_time, func1.time, func1.values)
        v2 = np.interp(t, func2.time, func2.values)

        fig, ax = plt.subplots()
        ax.plot(t, v1, label='func1(t + dt)')
        ax.plot(t, v2, label='func2(t)')
        ax.legend()

        plt.title(title)
        plt.show()


def main():
    f1, f2, gt_time_offset = generate_data()
    print('gt_time_offset:', gt_time_offset)

    calib = TimeCalibration()
    calib.least_squares_calibrate(f1, f2)

    print('gt_time_offset:', gt_time_offset)
    print('estimated_time_offset:', calib.variable_time)


if __name__ == "__main__":
    main()
