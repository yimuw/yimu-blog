import math
import matplotlib.pyplot as plt
import numpy as np

NON_LINEAR_EXPERIMENT = True

NUM_OF_SIM_DATA = 30


def plot_cov2(mu, cov):
    w, v = np.linalg.eig(cov)
    eclipse_axis = w * v
    t = np.linspace(0, 2 * np.pi, 100)
    circle = np.vstack([np.cos(t), np.sin(t)])
    eclipse = np.dot(eclipse_axis, circle)
    plt.plot(eclipse[0, :] + mu[0], eclipse[1, :] + mu[1])


def result_comparison(gt_states, estimated_states, estimated_cov, string_info=''):
    gstates = np.vstack(gt_states)
    estates = np.vstack(estimated_states)

    xy_norm = np.linalg.norm((gt_states - estates)[:, :2], axis=1)

    plt.figure()
    plt.subplot(1,2,1)
    plt.plot(xy_norm)
    plt.title('Method: {}    norm(x_diff, y_diff)'.format(string_info))

    plt.subplot(1,2,2)
    plt.plot(gstates[:, 0], gstates[:, 1], '-o')
    plt.plot(estates[:, 0], estates[:, 1], '-*')

    for i, cov in enumerate(estimated_cov):
        mu = estimated_states[i][:2]
        plot_cov2(mu, cov[:2, :2])

    plt.title('Method: {}    -o is the ground truth.  -* is the estimated'.format(string_info))
    plt.axis('equal')


class PointModel:
    """
    x = f(x)

    y = h(x)

    """
    DT = 0.1
    NUM_STATES = 5
    NUM_OBSERVATION = 2
    STATE_NAME = ['x', 'y', 'v', 'theta', 'theta_dot']

    @staticmethod
    def unpack_state(state):
        x, y, v, theta, theta_dot = state
        return x, y, v, theta, theta_dot

    def f(self, state):
        """
            x_next = f(x)
        """
        x, y, v, theta, theta_dot = self.unpack_state(state)
        return np.array([
            x + self.DT * math.cos(theta) * v,
            y + self.DT * math.sin(theta) * v,
            v,
            theta + self.DT * theta_dot,
            theta_dot
        ])
    
    def f_jacobian(self, state):
        """
            df/dx
        """
        x, y, v, theta, theta_dot = self.unpack_state(state)
        return np.array([
            [1, 0, self.DT * math.cos(theta), - self.DT * math.sin(theta) * v, 0],
            [0, 1, self.DT * math.sin(theta), self.DT * math.cos(theta) * v, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, self.DT],
            [0, 0, 0, 0, 1.]
        ])

    def f_weight(self):
        return np.diag([5, 5, 10, 100, 100])

    def f_cov(self):
        return np.linalg.inv(self.f_weight())

    def h(self, state):
        """
          z = h(x)
        """
        x, y, v, theta, theta_dot = self.unpack_state(state)
        if NON_LINEAR_EXPERIMENT:
            return np.array([x * x, y * y])
        else:
            return np.array([x, y])

    def h_jacobian(self, state):
        """
            dh/dx
        """
        x, y, v, theta, theta_dot = self.unpack_state(state)
        if NON_LINEAR_EXPERIMENT:
            return np.array([
                [2 * x, 0, 0, 0, 0],
                [0, 2 * y, 0, 0, 0],
            ])
        else:
            return np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
            ])

    def h_weight(self):
        return np.array([
            [5, 0],
            [0, 5]
        ])

    def h_cov(self):
        return np.linalg.inv(self.h_weight())


def get_model():
    return PointModel()


def generate_gt_data(init_state):
    model = get_model()

    gt_states = []
    gt_measurements = []

    state = init_state

    for step in range(NUM_OF_SIM_DATA):
        state = model.f(state)
        measurement = model.h(state)
        gt_states.append(state)
        gt_measurements.append(measurement)

    PLOT_STATE = False
    if PLOT_STATE:
        states = np.vstack(gt_states)
        plt.plot(states[:, 0], states[:, 1], '-o')
        plt.title('gt states')
        plt.show()

    return gt_states, gt_measurements