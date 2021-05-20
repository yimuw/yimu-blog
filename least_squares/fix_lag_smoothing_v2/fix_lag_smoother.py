import scipy.linalg as linalg
import numpy as np
import profiler
import fix_lag_types as t
import copy
from collections import deque

# lag = 1.
# Similar to Kalman filter without noise
class FixLagSmoother:
    def __init__(self, prior_measurement, gps_measurement):
        # using deque (linked list) for head & tail lookup
        self.state_opt_vars = deque([t.State(prior_measurement.copy())])
        # mantain a looking between state idx and state in the deque
        self.state_idx_offset = 0

        # make sure the fixed-lagged graph invariant hold at the beginning
        self.odometry_measurements = deque()
        self.gps_measurements = deque([gps_measurement])

        # For prior cost, Jacobian is I. Direct assign for simplicity
        # Assuming Identity weight matrix.
        self.prior_hessian = np.identity(4)
        self.prior_mean = np.zeros((4,1))

        # config
        self.window_size = 3

        # result
        self.result = []


    def get_all_states(self):
        return np.vstack(self.result)

    def state_index_lookup(self, idx):
        return idx - self.state_idx_offset

    def extent_graph(self, odometry_measurement, gps_measurement):
        # extent the graph
        self.odometry_measurements.append(odometry_measurement)
        self.gps_measurements.append(gps_measurement)
        self.state_opt_vars.append(t.State(self.state_opt_vars[-1].variables.copy()))

    def construct_linear_system(self):
        """
            Adding new measurements. Insert a new state implicitly
        """
        size_state = t.State.size()
        num_states = len(self.state_opt_vars)
        size_opt_vars = size_state * num_states

        # normal equation
        lhs = np.zeros([size_opt_vars, size_opt_vars])
        rhs = np.zeros([size_opt_vars, 1])

        # Adding prior cost
        # update normal equation directly using quadratic intepretation.
        # otherwise, we need do to a Cholesky which is not efficient.
        lhs[:size_state, :size_state] += self.prior_hessian
        rhs[:size_state] += self.prior_mean

        # Odometry costs
        for odometry_measurement in self.odometry_measurements:
            state1_idx = self.state_index_lookup(odometry_measurement.state1_index)
            state1_var_idx = size_state * state1_idx
            state1 = self.state_opt_vars[state1_idx]

            state2_idx = self.state_index_lookup(odometry_measurement.state2_index)
            state2_var_idx = size_state * state2_idx
            state2 = self.state_opt_vars[state2_idx]

            odometry_cost = t.OdometryCost(state1, state2)

            cur_jacobi = np.zeros([odometry_cost.residual_size(), size_opt_vars])
            # print(cur_jacobi)
            cur_jacobi[:, state1_var_idx:state1_var_idx + odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state1()
            cur_jacobi[:, state2_var_idx:state2_var_idx + odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state2()

            cur_residual = odometry_cost.residual()

            # Using the linear property of the least squares costs
            lhs += cur_jacobi.T @ cur_jacobi
            rhs += cur_jacobi.T @ cur_residual

        # GPS costs
        for gps_measurement in self.gps_measurements:
            state_idx = self.state_index_lookup(gps_measurement.state_index)
            state_var_idx = size_state * state_idx
            state = self.state_opt_vars[state_idx]

            gps_cost = t.GPSCost(state, gps_measurement.gps)

            cur_jacobi = np.zeros([gps_cost.residual_size(), size_opt_vars])
            cur_jacobi[:, state_var_idx:state_var_idx + gps_cost.variable_size()] = gps_cost.jacobi_wrt_state()

            cur_residual = gps_cost.residual()

            # Using the linear property of the least squares costs
            lhs += cur_jacobi.T @ cur_jacobi
            rhs += cur_jacobi.T @ cur_residual

        return lhs, rhs

    def solve_and_update(self, lhs, rhs):
        # one step for linear system
        # Using Nonlinear Gaussian-Newton formulation, the result is dx 
        num_states = len(self.state_opt_vars)
        dx = np.linalg.solve(lhs, - rhs)
        dx = dx.reshape(num_states, t.State.size())

        # iterator for a deque
        for i, s in enumerate(self.state_opt_vars):
            s.variables += dx[i]

    @profiler.time_it
    def optimize_for_new_measurement(self, distance_measurement, gps_measurement):
        self.extent_graph(distance_measurement, gps_measurement)

        lhs, rhs = self.construct_linear_system()

        self.solve_and_update(lhs, rhs)
        
        if len(self.state_opt_vars) > self.window_size:
            self.marginalization_schur_impl()

        # save the result for the currrent state 
        self.result.append(self.state_opt_vars[-1].variables.copy())

    def construct_marginalization_equation(self):
        """
            get all costs connect to the state to be marginalized and construct the equation for marginalization.
            TODO: this can be a by-product from construct_linear_system. 
        """
        size_state = t.State.size()
        num_states = 2
        size_opt_vars = size_state * num_states

        # normal equation
        lhs = np.zeros([size_opt_vars, size_opt_vars])
        rhs = np.zeros([size_opt_vars, 1])

        # remove data related to state to be marginalized
        # using prior knowledge. not general.
        gps_to_be_marginalized = self.gps_measurements.popleft()
        odo_to_be_marginalized = self.odometry_measurements.popleft()

        # Prior cost
        lhs[:size_state, :size_state] += self.prior_hessian
        rhs[:size_state] += self.prior_mean

        # Odometry cost
        state1_idx = self.state_index_lookup(odo_to_be_marginalized.state1_index)
        state1_var_idx = size_state * state1_idx
        state1 = self.state_opt_vars[state1_idx]

        state2_idx = self.state_index_lookup(odo_to_be_marginalized.state2_index)
        state2_var_idx = size_state * state2_idx
        state2 = self.state_opt_vars[state2_idx]

        odometry_cost = t.OdometryCost(state1, state2)
        cur_jacobi = np.zeros([odometry_cost.residual_size(), size_opt_vars])
        cur_jacobi[:, state1_var_idx:state1_var_idx + odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state1()
        cur_jacobi[:, state2_var_idx:state2_var_idx + odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state2()

        cur_residual = odometry_cost.residual()

        # Using the linear property of the least squares costs
        lhs += cur_jacobi.T @ cur_jacobi
        rhs += cur_jacobi.T @ cur_residual

        # GPS costs
        state_idx = self.state_index_lookup(gps_to_be_marginalized.state_index)
        state_var_idx = size_state * state_idx
        state = self.state_opt_vars[state_idx]

        gps_cost = t.GPSCost(state, gps_to_be_marginalized.gps)

        cur_jacobi = np.zeros([gps_cost.residual_size(), size_opt_vars])
        cur_jacobi[:, state_var_idx:state_var_idx + gps_cost.variable_size()] = gps_cost.jacobi_wrt_state()

        cur_residual = gps_cost.residual()

        # Using the linear property of the least squares costs
        lhs += cur_jacobi.T @ cur_jacobi
        rhs += cur_jacobi.T @ cur_residual

        return lhs, rhs


    @profiler.time_it
    def marginalization_schur_impl(self):
        """
            Force schur ordering.
        """
        lhs, rhs = self.construct_marginalization_equation()

        size_state = t.State.size()
        # lhs = [A, B; C, D]
        A = lhs[:size_state, :size_state]
        B = lhs[:size_state, size_state:]
        C = lhs[size_state:, :size_state]
        D = lhs[size_state:, size_state:]
        # rhs = [b1, b2]
        b1 = rhs[:size_state]
        b2 = rhs[size_state:]
        A_inv = np.linalg.inv(A)

        # schur
        self.prior_hessian = D - C @ A_inv @ B
        self.prior_mean = b2 - C @ A_inv @ b1

        # pop tail
        self.state_opt_vars.popleft()
        self.state_idx_offset += 1

