import scipy.linalg as linalg
import numpy as np


class LinearDynamic:
    # TODO: change the API
    def jacobi_wrt_state(self, state, controls):
        return np.array([
            [1, 0],
            [0, 1.],
        ])

    def jacobi_wrt_controls(self, state, controls):
        return np.array([
            [2, 0],
            [0, 1.],
        ])

    def f_function(self, state, controls):
        x, y = state
        ux, uy = controls

        return np.array([x + 2 * ux, 
                         y + uy])

class NonlinearDynamic:
    # TODO: change the API
    def jacobi_wrt_state(self, state, controls):
        return np.array([
            [1, 0],
            [0, 1.],
        ])

    def jacobi_wrt_controls(self, state, controls):
        ux, uy = controls
        return np.array([
            [2 * ux, 0],
            [uy, ux],
        ])

    def f_function(self, state, controls):
        x, y = state
        ux, uy = controls

        return np.array([x + ux * ux, 
                         y + uy * ux])


class TargetCost:
    def __init__(self, state, prior):
        self.state = state.copy()
        self.prior = prior.copy()

    def residual(self):
        x, y = self.state
        px, py = self.prior
        return np.array([
            [x - px],
            [y - py],
        ])

    def cost(self):
        r = self.residual()
        return r.T @ r

    def jacobi(self):
        return np.identity(2)

    def weight(self):
        return np.identity(2)

    def quad_weight(self):
        J = self.jacobi()
        W = self.weight()
        return J.T @ W @ J

    def quad_mean(self):
        return self.prior