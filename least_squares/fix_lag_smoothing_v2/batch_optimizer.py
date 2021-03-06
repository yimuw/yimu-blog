import scipy.linalg as linalg
import numpy as np
import profiler
import fix_lag_types as t


class BatchOptimization:
    def __init__(self, states, odometry_measurements, gps_measurements, prior_measurement):
        assert(len(states) - 1 == len(odometry_measurements))
        assert(len(states) == len(gps_measurements))
        self.__states = states
        self.__odometry_measurements = odometry_measurements
        self.__gps_measurements = gps_measurements
        self.__prior_measurement = prior_measurement
        self.__num_states = len(states)

    @profiler.time_it
    def optimize(self):
        jacobi, r = self.construct_linear_system()

        # one step for linear system
        # Using Nonlinear Gaussian-Newton formulation, the result is dx
        lhs = jacobi.T @ jacobi
        rhs = jacobi.T @ r
        dx = np.linalg.solve(lhs, - rhs)
        dx = dx.reshape(self.__num_states, t.State.size())
        for i, s in enumerate(self.__states):
            s.variables += dx[i]

        return self.__states, self.covariance_estimation(lhs)

    def covariance_estimation(self, lhs):
        return np.linalg.inv(lhs)

    def construct_linear_system(self):
        size_state = t.State.size()
        size_variables = size_state * self.__num_states

        # Prior cost
        state_idx = 0
        state_var_idx = 0
        prior_cost = t.PriorCost(
            self.__states[state_idx], self.__prior_measurement)
        jacobi = np.zeros([prior_cost.residual_size(), size_variables])
        jacobi[:, state_var_idx:state_var_idx +
               prior_cost.variable_size()] = prior_cost.jacobi_wrt_state()
        residual = prior_cost.residual()

        # Odometry costs
        for odometry_measurement in self.__odometry_measurements:
            state1_idx = odometry_measurement.state1_index
            state1_var_idx = size_state * state1_idx
            state1 = self.__states[state1_idx]

            state2_idx = odometry_measurement.state2_index
            state2_var_idx = size_state * state2_idx
            state2 = self.__states[state2_idx]

            odometry_cost = t.OdometryCost(state1, state2)

            cur_jacobi = np.zeros(
                [odometry_cost.residual_size(), size_variables])
            cur_jacobi[:, state1_var_idx:state1_var_idx +
                       odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state1()
            cur_jacobi[:, state2_var_idx:state2_var_idx +
                       odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state2()

            cur_residual = odometry_cost.residual()

            jacobi = np.vstack([jacobi, cur_jacobi])
            residual = np.vstack([residual, cur_residual])

        # GPS costs
        for gps_measurement in self.__gps_measurements:
            state_idx = gps_measurement.state_index
            state_var_idx = size_state * state_idx
            state = self.__states[state_idx]

            gps_cost = t.GPSCost(state, gps_measurement.gps)

            cur_jacobi = np.zeros([gps_cost.residual_size(), size_variables])
            cur_jacobi[:, state_var_idx:state_var_idx +
                       gps_cost.variable_size()] = gps_cost.jacobi_wrt_state()

            cur_residual = gps_cost.residual()

            jacobi = np.vstack([jacobi, cur_jacobi])

            residual = np.vstack([residual, cur_residual])

        return jacobi, residual
