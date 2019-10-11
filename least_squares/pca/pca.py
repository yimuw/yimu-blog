import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt


def euler_angle_to_rotation(yaw, pitch, roll):
    Rz = np.array([
        [cos(yaw), -sin(yaw), 0.],
        [sin(yaw), cos(yaw), 0.],
        [0, 0, 1.],
    ])
    Ry = np.array([
        [cos(pitch), -0., sin(pitch)],
        [0., 1., 0.],
        [-sin(pitch), 0, cos(pitch)],
    ])
    Rx = np.array([
        [1., 0., 0.],
        [0., cos(roll), -sin(roll)],
        [0, sin(roll), cos(roll)],
    ])

    return Rz @ Ry @ Rx

def generate_point_cloud():
    mean = np.array([0, 0, 0.])
    
    R = euler_angle_to_rotation(0.4, -1., 6.6666)
    R = euler_angle_to_rotation(0., 0., 0.)
    # Transformation for covariance
    cov = R @ np.diag([1, 1, 2.]) @ R.transpose()
    points = np.random.multivariate_normal(mean, cov, size=500)

    return points.transpose()


def skew(w):
    wx, wy, wz = w
    return np.array([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0.],
    ])


class PCA_SO3:
    def __init__(self, points):
        self.points = points
        self.points_covariance = points @ points.transpose() / points.shape[1]
        # print(self.points_covariance)
        self.var_SO3 = np.identity(3)
        self.var_scales = np.array([[1., 1., 1]]).transpose()


    def residaul(self):
        residual = (self.points_covariance - self.var_SO3 * self.var_scales * self.var_SO3.transpose()).flatten()
        return residual

    def cost(self):
        r = self.residaul()
        return r.transpose() @ r

    def jacobian(self)


def main():
    points = generate_point_cloud()
    pca = PCA_SO3(points)
    print("pca.cost()", pca.cost())


if __name__ == "__main__":
    main()
