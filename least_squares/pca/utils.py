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

    
def skew(w):
    wx, wy, wz = w
    return np.array([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0.],
    ])

def so3_exp(w):
    theta = np.linalg.norm(w)
    # Approximation when theta is small
    if(abs(theta) < 1e-8):
        return np.identity(3) + skew(w)

    normalized_w  = w / theta
    K = skew(normalized_w)
    # Rodrigues
    R = np.identity(3) + sin(theta) * K + (1 - cos(theta)) * K @ K
    np.testing.assert_almost_equal(R @ R.transpose(), np.identity(3))
    
    return R