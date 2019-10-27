import scipy.linalg as linalg 
import numpy as np
import profiler


class State:
    def __init__(self, variables):
        self.variables = variables
    
    def unpack_state(self):
        x, y = self.variables
        return x, y


class DistanceMeasurement:
    def __init__(self, state1, state2, distance):
        # Dangeours
        self.state1 = state1
        self.state2 = state2
        self.distance = distance.copy()
    
    def residual(self):
        x1, y1 = self.state1.unpack_state()
        x2, y2 = self.state2.unpack_state()
        dx, dy = self.distance
        return np.array([
            [x2 - x1 - dx],
            [y2 - y1 - dy],
        ])
    
    def jacobi_wrt_state1(self):
        return -np.identity(2)

    def jacobi_wrt_state2(self):
        return np.identity(2)

    def residual_size(self):
        return 2

    def variable_size(self):
        return 2

class PriorMeasurement:
    def __init__(self, state, prior):
        self.state = state
        self.prior = prior.copy()
    
    def residual(self):
        x, y = self.state.unpack_state()
        px, py = self.prior
        return np.array([
            [x - px],
            [y - py],
        ])

    def jacobi(self):
        return np.identity(2)

    def residual_size(self):
        return 2

    def variable_size(self):
        return 2
