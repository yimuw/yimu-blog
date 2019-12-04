import math
import matplotlib.pyplot as plt
import numpy as np

from model import *


class BatchLeastSqaures:
    """
       cost(x1, x2, ..., xn) = sum ||x - f(x-1)||^2_{f_weight} + sum ||z - h(x)||^2_weight
      Note: it is not the equality contrainted version
    """

    def __init__(self, prior_state=None, prior_cov=None):
        """
        technically, only prior is need for graph based filter
        :param prior_state:
        :param prior_cov:
        """
        self.prior_state = prior_state

    @staticmethod
    def get_state_i(varibles_vector, idx):
        model = get_model()
        size_state = model.NUM_STATES

        return varibles_vector[idx * size_state: (idx + 1) * size_state]

    def construct_linear_system(self, current_variables, measurements):

        model = get_model()

        # size of a state.
        size_state = model.NUM_STATES
        # number of nodes (states) in a graph
        num_nodes = len(measurements)
        # total number of variables for all nodes
        num_varibles = size_state * num_nodes
        # size of a measurement vector
        size_measurement = model.NUM_OBSERVATION
        # total number of measurements
        num_measurements = size_measurement * len(measurements)
        # there is a kinematic link between 2 measurements. So number is n -1
        # It is a full constrain for all state since kinematics model is given
        odometry_equ_size = size_state
        num_odometry_equations = (num_nodes - 1) * odometry_equ_size

        observation_equ_size = size_measurement
        num_observation_equations = len(measurements) * observation_equ_size

        # For asserts
        num_equations = num_odometry_equations + num_observation_equations

        # Jacobian. row is number of equations (residuals). Cols is the number of variables
        J = np.zeros([num_equations, num_varibles])
        # Weight. weight for residuals. Typically very sparse or diagonal.
        W = np.zeros([num_equations, num_equations])
        # Residual of the least sqr system
        r = np.zeros([num_equations, 1]).squeeze()

        # keep track of equations, a better implementation is associated a index for every factor.
        n = 0

        # update odometry equations
        for nidx in range(len(measurements) - 1):
            this_node_varible_idx = nidx * size_state
            next_node_varible_idx = (nidx + 1) * size_state
            assert nidx + 1 < len(measurements)

            this_state = self.get_state_i(current_variables, nidx)
            next_staet = self.get_state_i(current_variables, nidx + 1)

            # residual is "r = f(x1) - x2" => derivatives are "dr/dx1 = df(x1)", "dr/dx2 = -I"
            # model constrain
            # df / dx1
            J[n:n + odometry_equ_size, this_node_varible_idx:this_node_varible_idx + size_state] \
                = model.f_jacobian(this_state)
            # df / dx2
            J[n:n + odometry_equ_size, next_node_varible_idx: next_node_varible_idx + size_state] \
                = -np.identity(size_state)
            r[n:n + odometry_equ_size] = model.f(this_state) - next_staet
            W[n:n + odometry_equ_size, n:n + odometry_equ_size] = model.f_weight()
            n += odometry_equ_size

        assert n == num_odometry_equations

        for midx, measurement in enumerate(measurements):
            this_state = self.get_state_i(current_variables, midx)
            variable_idx = midx * size_state
            # observation
            J[n: n + observation_equ_size, variable_idx: variable_idx + size_state] += model.h_jacobian(this_state)
            r[n: n + observation_equ_size] = model.h(this_state) - measurement
            W[n: n + observation_equ_size, n: n + observation_equ_size] = model.h_weight()

            n += observation_equ_size

        assert n == num_equations

        J_transpose_dot_weight = np.dot(J.T, W)
        A = np.dot(J_transpose_dot_weight, J)
        b = np.dot(J_transpose_dot_weight, r)

        SPY_MATRIX = False
        if SPY_MATRIX:
            plt.spy(J, precision=1e-2)
            plt.title('J')
            plt.figure()
            plt.spy(A, precision=1e-2)
            plt.title('A.T * A')
            plt.figure()
            plt.spy(W)
            plt.title('Weight')
            plt.show()

        return A, b

    def init_variables(self, num_nodes):
        model = get_model()
        # total number of variables for all nodes
        state_size = model.NUM_STATES
        num_variables = state_size * num_nodes

        variables = np.zeros([num_variables, 1]).squeeze()

        state_pred = self.prior_state
        for i in range(num_nodes):
            var_idx = i * state_size
            state_pred = model.f(state_pred)
            variables[var_idx: var_idx + state_size] = state_pred

        return variables

    def filter(self, measurements):
        """
        Assume there is a kinematic link between 2 measurements
        :param measurements:
        :return:
        """
        variables = self.init_variables(len(measurements))

        hessian = None
        for newton_iter in range(50):
            # print(('newton iter :', newton_iter))
            A, b = self.construct_linear_system(variables, measurements)
            dx = np.linalg.solve(A, b)

            LAMBDA = 1
            variables -= LAMBDA * dx

            # by construction, A is the hessian for least sqr problem
            hessian = A

            if np.linalg.norm(dx) < 1e-8:
                break

        optimized_states = variables
        full_covariance = np.linalg.inv(hessian)

        return self.format_to_states_and_covs(optimized_states, full_covariance)

    def format_to_states_and_covs(self, optimized_states, full_covariance):
        model = get_model()
        state_size = model.NUM_STATES
        num_nodes = int(optimized_states.shape[0] / state_size)

        states = []
        covs = []
        for i in range(num_nodes):
            this_state = self.get_state_i(optimized_states, i)
            states.append(this_state)

            index_start = i * state_size
            cov = full_covariance[index_start: index_start + state_size, index_start: index_start + state_size]
            covs.append(cov)

        return states, covs


def run_batch_least_sqr(init_state):
    gt_states, gt_measurements = generate_gt_data(init_state)

    prior_state = np.array([0.5, 0.5, 0.1, 0, 0])
    # prior_state = init_state
    prior_cov = np.diag([1, 1, 1, 1., 1.])

    graph_filter = BatchLeastSqaures(prior_state, prior_cov)
    estimated_states, estimated_cov = graph_filter.filter(gt_measurements)

    result_comparison(gt_states, estimated_states, estimated_cov, 'Graph Optimization')
