import math
import matplotlib.pyplot as plt
import numpy as np

from model import *


class KalmanLeastSqaures:
    """
    cost(x) = ||F^-1 x - x_pre||^2_{FWF^T + Q}^{-1} + ||z - Hx||^2_R^{-1}
    """
    def __init__(self, prior_state, prior_cov):
        self.state = prior_state
        self.cov = prior_cov

    def filter(self, measurement):
        prior_state = self.state
        prior_cov = self.cov

        self.least_sqr_optimization(prior_state, prior_cov, measurement)

    def compute_jacobian_and_residual(self, current_varialbes,
                                      prior_state, prior_cov, measurement):
        model = get_model()
        num_state = model.NUM_STATES
        num_variables = num_state
        num_measurements = model.NUM_OBSERVATION

        num_prior_equations = model.NUM_STATES
        num_observation_equations = model.NUM_OBSERVATION
        num_equations = num_prior_equations + num_observation_equations

        jacobian = np.zeros([num_equations, num_variables])
        weight = np.zeros([num_equations, num_equations])
        residual = np.zeros([num_equations, 1]).squeeze()

        n = 0
        f_jacobian_prev = model.f_jacobian(prior_state)
        # prior
        jacobian[n: num_prior_equations, :] = np.identity(num_state)
        residual[n:num_prior_equations] = current_varialbes - model.f(prior_state)
        weight[n:num_prior_equations, n:num_prior_equations] \
             = np.linalg.inv(f_jacobian_prev @ prior_cov @ f_jacobian_prev.T + model.f_cov())
        n += num_prior_equations

        # observation
        jacobian[n: n + num_observation_equations, :] = model.h_jacobian(current_varialbes)
        residual[n: n + num_observation_equations] = model.h(current_varialbes) - measurement
        weight[n: n + num_observation_equations, n: n + num_observation_equations] = model.h_weight()
        n += num_observation_equations

        assert n  == num_equations

        return jacobian, residual, weight
    
    def construct_normal_equation(self, jacobian, residual, weight):
        J_transpose_dot_weight = np.dot(jacobian.T, weight)
        norm_equation_left = np.dot(J_transpose_dot_weight, jacobian)
        norm_equation_right = - np.dot(J_transpose_dot_weight, residual)
        return norm_equation_left, norm_equation_right


    def least_sqr_optimization(self, prior_state, prior_cov, measurement):
        model = get_model()
        variables = model.f(prior_state)

        for iter in range(10):
            jacobian, residual, weight \
                = self.compute_jacobian_and_residual(variables, prior_state, prior_cov, measurement)

            norm_equation_left, norm_equation_right \
                = self.construct_normal_equation(jacobian, residual, weight)

            dx = np.linalg.solve(norm_equation_left, norm_equation_right)

            LAMBDA = 1

            variables += LAMBDA * dx

            if np.linalg.norm(dx) < 1e-8:
                break

        self.state = variables
        self.cov = np.linalg.inv(norm_equation_left)


def run_kalman_plus_filter(init_state):
    gt_states, gt_measurements = generate_gt_data(init_state)

    prior_state = np.array([0.5, 0.5, 0, 0, 0])
    # prior_state = init_state
    prior_cov = np.diag([1, 1, 1, 1., 1.])

    kalman_p_filter = KalmanLeastSqaures(prior_state, prior_cov)

    estimated_states = []
    estimated_cov = []

    for i, m in enumerate(gt_measurements):
        kalman_p_filter.filter(m)

        estimated_states.append(kalman_p_filter.state)
        estimated_cov.append(kalman_p_filter.cov)

    result_comparison(gt_states, estimated_states, estimated_cov, 'kalman least sqaures')

