import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt
from utils import euler_angle_to_rotation

def icp_residual_yaw_pitch_roll(point_src, point_target, yaw_pitch_roll):
    R = euler_angle_to_rotation(*yaw_pitch_roll)
    residual =R @ point_src - point_target
    # [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, ...]
    residual = residual.flatten('F')
    return residual

def compute_yaw_pitch_roll_jacobian_numurical(point_src, point_target, yaw_pitch_roll):
    DELTA = 1e-6

    num_residuals = point_src.size
    num_params = 3

    jacobian = np.zeros([num_residuals, num_params])
    curret_params = yaw_pitch_roll.copy()
    for p_idx in range(3):
        params_plus = curret_params.copy()
        params_plus[p_idx] += DELTA
        residual_plus = icp_residual_yaw_pitch_roll(point_src, point_target, params_plus)

        params_minus = curret_params.copy()
        params_minus[p_idx] -= DELTA
        residual_minus = icp_residual_yaw_pitch_roll(point_src, point_target, params_minus)

        dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
        jacobian[:, p_idx] = dr_dpidx

    residual_cur_params = icp_residual_yaw_pitch_roll(point_src, point_target, yaw_pitch_roll)

    return jacobian, residual_cur_params

def icp_yaw_pitch_roll_numirical(point_src, point_target):
    yaw_pitch_roll = np.array([0, 0, 0.])

    for iter in range(10):
        jacobi, b = compute_yaw_pitch_roll_jacobian_numurical(point_src, point_target, yaw_pitch_roll)
        delta = np.linalg.solve(jacobi.transpose() @ jacobi, -jacobi.transpose() @ b)
        yaw_pitch_roll += delta

        #print('jocobian:', jacobi)
        #print('b: ', b)
        print('iter: ', iter, ' cost:', b.transpose() @ b)
        #print('current params: ', yaw_pitch_roll)
