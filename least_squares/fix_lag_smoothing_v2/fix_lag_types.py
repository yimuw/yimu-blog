import scipy.linalg as linalg
import numpy as np
import profiler


class State:
    def __init__(self, variables):
        self.variables = variables

    def unpack_state(self):
        x, y, vx, vy = self.variables
        return x, y, vx, vy

    def predict(self):
        x, y, vx, vy = self.variables
        predicted_state = np.array([x + vx, y + vy, vx, vy])
        return State(predicted_state)

    def __repr__(self):
        return self.variables.__repr__()

    @staticmethod
    def size():
        return 4


class OdometryMeasurement:
    """
        Help the mapping between a state and the state's index in the Hessian of the optimization problem.
    """

    def __init__(self, state1_index, state2_index):
        # Dangeours
        self.state1_index = state1_index
        self.state2_index = state2_index

    def __repr__(self):
        return '({}, {})'.format(self.state1_index, self.state2_index)


class OdometryCost:
    def __init__(self, state1, state2):
        # Dangeours
        self.state1 = state1
        self.state2 = state2

    def residual(self):
        x1, y1, vx1, vy1 = self.state1.unpack_state()
        x2, y2, vx2, vy2 = self.state2.unpack_state()
        # s2 - pred(s1)
        return np.array([
            [x2 - x1 - vx1],
            [y2 - y1 - vy1],
            [vx2 - vx1],
            [vy2 - vy1],
        ])

    def jacobi_wrt_state1(self):
        j = - np.identity(4)
        j[0, 2] = -1.
        j[1, 3] = -1.
        return j

    def jacobi_wrt_state2(self):
        return np.identity(4)

    @staticmethod
    def residual_size():
        return 4

    @staticmethod
    def variable_size():
        return 4


class GPSMeasurement:
    def __init__(self, state_index, gps):
        self.state_index = state_index
        self.gps = gps.copy()

    def __repr__(self):
        return '({}-{})'.format(self.state_index, self.gps)


class GPSCost:
    def __init__(self, state, gps):
        self.state = state
        self.gps = gps.copy()

    def residual(self):
        x, y, _, _ = self.state.unpack_state()
        gx, gy = self.gps
        return np.array([
            [x - gx],
            [y - gy],
        ])

    def jacobi_wrt_state(self):
        j = np.zeros([2, 4])
        j[0:2, 0:2] = np.identity(2)
        return j

    @staticmethod
    def residual_size():
        return 2

    @staticmethod
    def variable_size():
        return 4


class PriorCost:
    def __init__(self, state, prior):
        self.state = state
        self.prior = prior.copy()

    def residual(self):
        return (self.state.variables - self.prior).reshape([4, 1])

    def jacobi_wrt_state(self):
        return np.identity(4)

    @staticmethod
    def residual_size():
        return 4

    @staticmethod
    def variable_size():
        return 4
