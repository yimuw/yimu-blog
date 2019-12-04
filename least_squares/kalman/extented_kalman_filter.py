import math
import matplotlib.pyplot as plt
import numpy as np

from model import *


class ExtentedKalmanFilter:

    def __init__(self, prior_state, prior_cov):
        self.state = prior_state
        self.cov = prior_cov

    def predict(self):
        model = get_model()

        self.state = model.f(self.state)

        f_jacobian = model.f_jacobian(self.state)
        self.cov = f_jacobian @ self.cov @ f_jacobian.T + model.f_cov()

    def update(self, measurement):
        model = get_model()
        from numpy.linalg import inv

        h_jacobian = model.h_jacobian(self.state)

        predict_cov_inv = inv(self.cov)
        H_cov_inv = inv(model.h_cov())
        self.cov = inv(predict_cov_inv + h_jacobian.T @ H_cov_inv @ h_jacobian)

        term1 = predict_cov_inv @ self.state + h_jacobian.T @ H_cov_inv @ measurement
        self.state = np.dot(self.cov, term1)

    def filter(self, measurement):
        self.predict()
        self.update(measurement)


def run_extented_kalman_filter(init_state):
    gt_states, gt_measurements = generate_gt_data(init_state)

    prior_state = np.array([0.5, 0.5, 0, 0, 0])
    prior_cov = np.diag([1, 1, 1, 1., 1.])

    kalman_filter = ExtentedKalmanFilter(prior_state, prior_cov)

    estimated_states = []
    estimated_cov = []

    for i, m in enumerate(gt_measurements):
        kalman_filter.filter(m)

        estimated_states.append(kalman_filter.state)
        estimated_cov.append(kalman_filter.cov)

    result_comparison(gt_states, estimated_states, estimated_cov, 'Kalman filter')
