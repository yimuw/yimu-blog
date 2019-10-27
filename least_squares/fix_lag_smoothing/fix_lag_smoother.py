import scipy.linalg as linalg 
import numpy as np
import profiler
import fix_lag_types as t


# lag = 1. 
# Similar to Kalman filter without noise
class FixLagSmoother:
    def __init__(self, prior):
        self.state = prior
        # Because I didn't specify weight, so weights are indentity
        self.state_weight = np.identity(2)

    @profiler.time_it
    def optimize_for_new_measurement(self, distance_measurement):
        jacobi, r = self.construct_linear_system(distance_measurement)
        # one step for linear system
        x = np.linalg.solve(jacobi.T @ jacobi, - jacobi.T @ r)

        return x

    def construct_linear_system(self):
        size_prior_residual = 2
        size_distance_residual = 2 * (self.num_states - 1)
        size_residual = size_distance_residual + size_prior_residual

        size_variables = 2 * self.num_states

        jacobi = np.zeros([size_residual, size_variables])
        residual = np.zeros([size_residual, 1])

        residual_index = 0
        # Add prior to first state. Not a good design.
        pm = t.PriorMeasurement(self.states[0], self.states[0].variables)
        jacobi[residual_index:residual_index + pm.residual_size(), 0:0 + pm.variable_size()] = pm.jacobi()
        residual[residual_index:residual_index + pm.residual_size()] = pm.residual()
        residual_index += pm.residual_size()

        # No distance residual for the last state
        for state_index in range(self.num_states - 1):
            state1_idx = state_index
            state1_var_idx = 2 * state1_idx
            state1 = self.states[state_index]

            state2_idx = state_index + 1
            state2_var_idx = 2 * state2_idx
            state2 = self.states[state2_idx]

            distance = self.distances[state_index]

            dm = t.DistanceMeasurement(state1, state2, distance)

            jacobi[residual_index:residual_index + dm.residual_size(), \
                   state1_var_idx:state1_var_idx + dm.variable_size()] = dm.jacobi_wrt_state1()
            jacobi[residual_index:residual_index + dm.residual_size(), \
                   state2_var_idx:state2_var_idx + dm.variable_size()] = dm.jacobi_wrt_state2()
            residual[residual_index:residual_index + dm.residual_size()] = dm.residual()
            residual_index += dm.residual_size()

        assert(residual_index == self.num_states * 2)
        return jacobi, residual