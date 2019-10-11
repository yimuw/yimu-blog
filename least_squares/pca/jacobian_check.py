import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt


def skew(w):
    wx, wy, wz = w
    return np.array([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0.],
    ])


class JacobianCheck:
    def __init__(self):

        # print(self.points_covariance)

        self.A = np.random.rand(3,3)
        self.var = np.array([0, 0, 0.])

    def function(A, w):
        return A @ skew(w)

    def df_dvar(self):
        jacobian = np.zeros([3,3,3])
        cur_var = self.var.copy()
        
        DELTA = 1e-6
        for var_idx in range(3):
            var_plus = cur_var.copy()
            var_plus[var_idx] += DELTA
            f_plus = JacobianCheck.function(self.A, var_plus)

            var_minus = cur_var.copy()
            var_minus[var_idx] -= DELTA
            f_minus = JacobianCheck.function(self.A, var_minus)

            jacobian[:,:,var_idx] = (f_plus - f_minus) / (2 * DELTA)
            print('var_idx:', var_idx)
            print('j:', jacobian[:, :, var_idx])

        return jacobian

    def df_dvar_analytic(self):
        # f = A * W
        # dfdw = A * (dW*dw)
        #      = [ (A * (G1)) * w1, (A * G2) * w2, (A * G3) * w3]
        jacobian = np.zeros([3,3,3])
        G1 = np.array([
            [0, 0, 0],
            [0, 0, -1],
            [0, 1, 0.],
        ])
        G2 = np.array([
            [0, 0, 1],
            [0, 0, 0],
            [-1, 0, 0.],
        ])
        G3 = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0.],
        ])
        jacobian[:, :, 0] = self.A @ G1
        jacobian[:, :, 1] = self.A @ G2
        jacobian[:, :, 2] = self.A @ G3

        for var_idx in range(3):
            print('var_idx:', var_idx)
            print('j:', jacobian[:, :, var_idx])


def main():
    jc = JacobianCheck()
    jc.df_dvar()
    jc.df_dvar_analytic()


if __name__ == "__main__":
    main()
