import scipy.linalg as linalg
import numpy as np
import profiler
import fix_lag_types as t
import copy
from collections import deque

UNBIAS_ESTIMATION = True
MARGINALIZATION_METHOD = 'gaussian'


class FixLagSmoother:
    def __init__(self, prior_measurement, gps_measurement):
        # using deque (linked list) for head & tail lookup
        self.__state_opt_vars = deque([t.State(prior_measurement.copy())])
        # mantain a lookup between state idx and state in the deque
        self.__state_idx_offset = 0

        # make sure the fixed-lagged graph invariant hold at the beginning
        self.__odometry_measurements = deque()
        self.__gps_measurements = deque([gps_measurement])

        # For prior cost, Jacobian is I. Direct assign for simplicity
        # Assuming Identity weight matrix.
        self.__prior_hessian = np.identity(4)
        self.__prior_mean = np.zeros((4, 1))

        # config
        self.__window_size = 5

        # result for easy query
        self.__result = []
        self.__diag_covs = []

    def get_all_states(self):
        return np.vstack(self.__result)

    def get_diag_cov(self):
        return np.vstack(self.__diag_covs)

    def state_index_lookup(self, idx):
        return idx - self.__state_idx_offset

    def extent_graph(self, odometry_measurement, gps_measurement):
        # extent the graph
        self.__odometry_measurements.append(odometry_measurement)
        self.__gps_measurements.append(gps_measurement)
        self.__state_opt_vars.append(
            t.State(self.__state_opt_vars[-1].variables.copy()))

    def construct_linear_system(self):
        """
            Adding new measurements. Insert a new state implicitly
        """
        size_state = t.State.size()
        num_states = len(self.__state_opt_vars)
        size_opt_vars = size_state * num_states

        # normal equation
        lhs = np.zeros([size_opt_vars, size_opt_vars])
        rhs = np.zeros([size_opt_vars, 1])

        # Adding prior cost
        # update normal equation directly using quadratic intepretation.
        # otherwise, we need do to a Cholesky which is not efficient.
        lhs[:size_state, :size_state] += self.__prior_hessian
        rhs[:size_state] += self.__prior_mean

        # Odometry costs
        for odometry_measurement in self.__odometry_measurements:
            state1_idx = self.state_index_lookup(
                odometry_measurement.state1_index)
            state1_var_idx = size_state * state1_idx
            state1 = self.__state_opt_vars[state1_idx]

            state2_idx = self.state_index_lookup(
                odometry_measurement.state2_index)
            state2_var_idx = size_state * state2_idx
            state2 = self.__state_opt_vars[state2_idx]

            odometry_cost = t.OdometryCost(state1, state2)

            cur_jacobi = np.zeros(
                [odometry_cost.residual_size(), size_opt_vars])
            cur_jacobi[:, state1_var_idx:state1_var_idx +
                       odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state1()
            cur_jacobi[:, state2_var_idx:state2_var_idx +
                       odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state2()

            cur_residual = odometry_cost.residual()

            # Using the linear property of the least squares costs
            lhs += cur_jacobi.T @ cur_jacobi
            rhs += cur_jacobi.T @ cur_residual

        # GPS costs
        for gps_measurement in self.__gps_measurements:
            state_idx = self.state_index_lookup(gps_measurement.state_index)
            state_var_idx = size_state * state_idx
            state = self.__state_opt_vars[state_idx]

            gps_cost = t.GPSCost(state, gps_measurement.gps)

            cur_jacobi = np.zeros([gps_cost.residual_size(), size_opt_vars])
            cur_jacobi[:, state_var_idx:state_var_idx +
                       gps_cost.variable_size()] = gps_cost.jacobi_wrt_state()

            cur_residual = gps_cost.residual()

            # Using the linear property of the least squares costs
            lhs += cur_jacobi.T @ cur_jacobi
            rhs += cur_jacobi.T @ cur_residual

        return lhs, rhs

    def covariance_estimation(self, lhs):
        return np.linalg.inv(lhs)

    def solve_and_update(self, lhs, rhs):
        # one step for linear system
        # Using Nonlinear Gaussian-Newton formulation, the __result is dx
        num_states = len(self.__state_opt_vars)
        dx = np.linalg.solve(lhs, - rhs)
        dx = dx.reshape(num_states, t.State.size())

        # No practical for nonlinear case
        if UNBIAS_ESTIMATION:
            for i in range(len(self.__state_opt_vars)):
                state_idx = self.__state_idx_offset + i
                if state_idx == len(self.__result):
                    self.__result.append(None)
                self.__result[state_idx] = self.__state_opt_vars[i].variables + dx[i]
        else:
            # iterator for a deque
            for i, s in enumerate(self.__state_opt_vars):
                s.variables += dx[i]

                state_idx = self.__state_idx_offset + i
                if state_idx == len(self.__result):
                    self.__result.append(None)
                self.__result[state_idx] = self.__state_opt_vars[i].variables

        cov = self.covariance_estimation(lhs)
        for i in range(len(self.__state_opt_vars)):
            state_idx = self.__state_idx_offset + i
            if state_idx == len(self.__diag_covs):
                self.__diag_covs.append(None)
            size_state = t.State.size()
            self.__diag_covs[state_idx] = np.diag(
                cov[i*size_state:(i+1)*size_state, i*size_state:(i+1)*size_state])

    @profiler.time_it
    def optimize_for_new_measurement(self, distance_measurement, gps_measurement):
        self.extent_graph(distance_measurement, gps_measurement)

        lhs, rhs = self.construct_linear_system()

        self.solve_and_update(lhs, rhs)

        if len(self.__state_opt_vars) > self.__window_size:
            if MARGINALIZATION_METHOD == 'schur':
                self.marginalization_schur_impl()
            elif MARGINALIZATION_METHOD == 'gaussian':
                self.marginalization_gaussian_impl()
            else:
                assert False, "unknow marginalization method"

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
        gps_to_be_marginalized = self.__gps_measurements.popleft()
        odo_to_be_marginalized = self.__odometry_measurements.popleft()

        # Prior cost
        lhs[:size_state, :size_state] += self.__prior_hessian
        rhs[:size_state] += self.__prior_mean

        # Odometry cost
        state1_idx = self.state_index_lookup(
            odo_to_be_marginalized.state1_index)
        state1_var_idx = size_state * state1_idx
        state1 = self.__state_opt_vars[state1_idx]

        state2_idx = self.state_index_lookup(
            odo_to_be_marginalized.state2_index)
        state2_var_idx = size_state * state2_idx
        state2 = self.__state_opt_vars[state2_idx]

        odometry_cost = t.OdometryCost(state1, state2)
        cur_jacobi = np.zeros([odometry_cost.residual_size(), size_opt_vars])
        cur_jacobi[:, state1_var_idx:state1_var_idx +
                   odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state1()
        cur_jacobi[:, state2_var_idx:state2_var_idx +
                   odometry_cost.variable_size()] = odometry_cost.jacobi_wrt_state2()

        cur_residual = odometry_cost.residual()

        # Using the linear property of the least squares costs
        lhs += cur_jacobi.T @ cur_jacobi
        rhs += cur_jacobi.T @ cur_residual

        # GPS costs
        state_idx = self.state_index_lookup(gps_to_be_marginalized.state_index)
        state_var_idx = size_state * state_idx
        state = self.__state_opt_vars[state_idx]

        gps_cost = t.GPSCost(state, gps_to_be_marginalized.gps)

        cur_jacobi = np.zeros([gps_cost.residual_size(), size_opt_vars])
        cur_jacobi[:, state_var_idx:state_var_idx +
                   gps_cost.variable_size()] = gps_cost.jacobi_wrt_state()

        cur_residual = gps_cost.residual()

        # Using the linear property of the least squares costs
        lhs += cur_jacobi.T @ cur_jacobi
        rhs += cur_jacobi.T @ cur_residual

        return lhs, rhs

    @profiler.time_it
    def marginalization_schur_impl(self):
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
        self.__prior_hessian = D - C @ A_inv @ B
        self.__prior_mean = b2 - C @ A_inv @ b1

        # pop tail
        self.__state_opt_vars.popleft()
        self.__state_idx_offset += 1

    @profiler.time_it
    def marginalization_gaussian_impl(self):
        lhs, rhs = self.construct_marginalization_equation()

        size_state = t.State.size()

        lhs_LU = linalg.lu_factor(lhs)
        covariance = linalg.lu_solve(lhs_LU, np.identity(size_state * 2))
        mean_covariance_form = covariance @ rhs

        self.__prior_hessian = np.linalg.inv(
            covariance[size_state:, size_state:])
        # This is tricky
        # get it by equating ||Ax + b||^2 = ||x + mu ||^2_W
        self.__prior_mean = self.__prior_hessian @ mean_covariance_form[size_state:]

        # pop tail
        self.__state_opt_vars.popleft()
        self.__state_idx_offset += 1
