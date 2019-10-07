import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
from copy import deepcopy

import utils


def V_operator(w):
    w_skew = utils.skew(w)
    theta = np.linalg.norm(w)
    if(abs(theta) < 1e-7):
        return np.identity(3) + w_skew / 2.
    
    V = np.identity(3) + (1. - cos(theta)) / (theta * theta) * w_skew + \
        + (theta - sin(theta)) / (theta * theta * theta) * w_skew @ w_skew
    return V

def se3_exp(se3):
    w = se3[:3]
    t = se3[3:].reshape([3,1])

    SE3_mat = np.identity(4)
    SE3_mat[:3, :3] = utils.so3_exp(w)
    # I hate numpy
    SE3_mat[:3, 3] = (V_operator(w) @ t).flatten()

    return SE3_mat

class SE3:
    def __init__(self):
        self.T = np.identity(4)

    def right_add(self, se3):
        assert(se3.size == 6)
        ret = SE3()
        ret.T = self.T @ se3_exp(se3)
        return ret


def icp_se3_residual(point_src, point_target, variables_SE3):
    R = variables_SE3.T[:3 , :3]
    translation = variables_SE3.T[:3, 3].reshape([3,1])
    
    residual =R @ point_src - point_target + translation
    # [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, ...]
    residual = residual.flatten('F')
    return residual


# Can do a lambda to reduce code length
def compute_se3_jacobian_numurical(point_src, point_target, variables_SE3):
    DELTA = 1e-6

    num_residuals = point_src.size
    num_variables = 6

    jacobian = np.zeros([num_residuals, num_variables])

    se3 = np.array([0, 0, 0, 0, 0, 0.])
    curret_variables = deepcopy(variables_SE3)
    for p_idx in range(6):
        variables_plus = deepcopy(curret_variables)
        delta_vector = se3.copy()
        delta_vector[p_idx] += DELTA
        variables_plus = variables_plus.right_add(delta_vector)
        residual_plus = icp_se3_residual(point_src, point_target, variables_plus)

        variables_minus = deepcopy(curret_variables)
        delta_vector = se3.copy()
        delta_vector[p_idx] -= DELTA
        variables_minus = variables_minus.right_add(delta_vector)
        residual_minus = icp_se3_residual(point_src, point_target, variables_minus)

        dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
        jacobian[:, p_idx] = dr_dpidx

    residual_cur_variables = icp_se3_residual(point_src, point_target, curret_variables)

    return jacobian, residual_cur_variables

def icp_se3_numirical(point_src, point_target):
    variables = SE3()

    for iter in range(10):
        # Jocobi on so3
        jacobi, b = compute_se3_jacobian_numurical(point_src, point_target, variables)
        delta = np.linalg.solve(jacobi.transpose() @ jacobi, -jacobi.transpose() @ b)

        # Update on SO3
        variables = variables.right_add(delta)

        #print('jocobian:', jacobi)
        #print('b: ', b)
        print('iter: ', iter, ' cost:', b.transpose() @ b)
        #print('current variables: ', w_so3)