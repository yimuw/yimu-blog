import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt
from utils import skew, so3_exp


def icp_residual_so3(point_src, point_target, w_so3):
    R = so3_exp(w_so3)
    residual =R @ point_src - point_target
    # [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, ...]
    residual = residual.flatten('F')
    return residual

def compute_so3_jacobian_numurical(point_src, point_target, w_so3):
    DELTA = 1e-6

    num_residuals = point_src.size
    num_params = 3

    jacobian = np.zeros([num_residuals, num_params])
    curret_params = w_so3.copy()
    for p_idx in range(3):
        params_plus = curret_params.copy()
        params_plus[p_idx] += DELTA
        residual_plus = icp_residual_so3(point_src, point_target, params_plus)

        params_minus = curret_params.copy()
        params_minus[p_idx] -= DELTA
        residual_minus = icp_residual_so3(point_src, point_target, params_minus)

        dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
        jacobian[:, p_idx] = dr_dpidx

    residual_cur_params = icp_residual_so3(point_src, point_target, w_so3)

    return jacobian, residual_cur_params

def icp_so3_numirical(point_src, point_target):
    w_so3 = np.array([0, 0, 0.])

    for iter in range(10):
        jacobi, b = compute_so3_jacobian_numurical(point_src, point_target, w_so3)
        delta = np.linalg.solve(jacobi.transpose() @ jacobi, -jacobi.transpose() @ b)
        w_so3 += delta

        #print('jocobian:', jacobi)
        #print('b: ', b)
        print('iter: ', iter, ' cost:', b.transpose() @ b)
        #print('current params: ', w_so3)


def icp_residual_local_so3(point_src, point_target, R_current, w_so3_local):
    R = R_current @ so3_exp(w_so3_local) 
    residual =R @ point_src - point_target
    # [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, ...]
    residual = residual.flatten('F')
    return residual


# Can do a lambda to reduce code length
def compute_local_so3_jacobian_numurical(point_src, point_target, R_current):
    DELTA = 1e-6

    num_residuals = point_src.size
    num_params = 3

    jacobian = np.zeros([num_residuals, num_params])

    w_so3_local = np.array([0, 0, 0.])
    curret_params = w_so3_local.copy()
    for p_idx in range(3):
        params_plus = curret_params.copy()
        params_plus[p_idx] += DELTA
        residual_plus = icp_residual_local_so3(point_src, point_target, R_current, params_plus)

        params_minus = curret_params.copy()
        params_minus[p_idx] -= DELTA
        residual_minus = icp_residual_local_so3(point_src, point_target, R_current, params_minus)

        dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
        jacobian[:, p_idx] = dr_dpidx

    residual_cur_params = icp_residual_local_so3(point_src, point_target, R_current, w_so3_local)

    return jacobian, residual_cur_params

def icp_local_so3_numirical(point_src, point_target):
    w_so3_local = np.array([0, 0, 0.])
    R_current = np.identity(3)
    for iter in range(10):
        # Jocobi on so3
        jacobi, b = compute_local_so3_jacobian_numurical(point_src, point_target, R_current)
        delta = np.linalg.solve(jacobi.transpose() @ jacobi, -jacobi.transpose() @ b)

        # Update on SO3
        R_current = R_current @ so3_exp(delta)

        #print('jocobian:', jacobi)
        #print('b: ', b)
        print('iter: ', iter, ' cost:', b.transpose() @ b)
        #print('current params: ', w_so3)
