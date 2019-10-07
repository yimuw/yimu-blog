import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
from copy import deepcopy

import utils


class SO3AndTranslation:
    def __init__(self):
        self.R = np.identity(3)
        self.translation = np.array([[0, 0, 0.]]).transpose()

    def right_add(self, so3_local_and_translation):
        assert(so3_local_and_translation.size == 6)
        w_so3_local = so3_local_and_translation[:3]
        translation = so3_local_and_translation[3:].reshape([3,1])

        ret = SO3AndTranslation()
        ret.R = self.R @ utils.so3_exp(w_so3_local)
        ret.translation = self.translation + translation
        assert(ret.translation.size == 3)
        return ret


def icp_so3_and_translation_residual(point_src, point_target, variables_SO3_and_translation):
    R = variables_SO3_and_translation.R
    translation = variables_SO3_and_translation.translation
    residual =R @ point_src - point_target + translation
    # [p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, ...]
    residual = residual.flatten('F')
    return residual


# Can do a lambda to reduce code length
def compute_so3_and_translation_jacobian_numurical(point_src, point_target, variables_SO3_and_translation):
    DELTA = 1e-6

    num_residuals = point_src.size
    num_variables = 6

    jacobian = np.zeros([num_residuals, num_variables])

    so3_and_translation = np.array([0, 0, 0, 0, 0, 0.])
    curret_variables = deepcopy(variables_SO3_and_translation)
    for p_idx in range(6):
        variables_plus = deepcopy(curret_variables)
        delta_vector = so3_and_translation.copy()
        delta_vector[p_idx] += DELTA
        variables_plus = variables_plus.right_add(delta_vector)
        residual_plus = icp_so3_and_translation_residual(point_src, point_target, variables_plus)

        variables_minus = deepcopy(curret_variables)
        delta_vector = so3_and_translation.copy()
        delta_vector[p_idx] -= DELTA
        variables_minus = variables_minus.right_add(delta_vector)
        residual_minus = icp_so3_and_translation_residual(point_src, point_target, variables_minus)

        dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
        jacobian[:, p_idx] = dr_dpidx

    residual_cur_variables = icp_so3_and_translation_residual(point_src, point_target, curret_variables)

    return jacobian, residual_cur_variables

def icp_so3_and_translation_numirical(point_src, point_target):
    variables = SO3AndTranslation()

    for iter in range(10):
        # Jocobi on so3
        jacobi, b = compute_so3_and_translation_jacobian_numurical(point_src, point_target, variables)
        delta = np.linalg.solve(jacobi.transpose() @ jacobi, -jacobi.transpose() @ b)

        # Update on SO3
        variables = variables.right_add(delta)

        #print('jocobian:', jacobi)
        #print('b: ', b)
        print('iter: ', iter, ' cost:', b.transpose() @ b)
        #print('current variables: ', w_so3)