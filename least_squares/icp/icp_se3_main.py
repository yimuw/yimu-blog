import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt

import icp_so3_and_t
import icp_se3
import utils

def generate_point_cloud():
    points = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 1, 1],
        [1, 0, 1],
        [0, 1e1, 1e5],
    ])

    return points.transpose()

def generate_simulation_data():
    point_src = generate_point_cloud()
    R_gt = utils.euler_angle_to_rotation(0.84, pi, pi)
    T_gt = np.array([[1, 10, 100]]).transpose()

    point_target = R_gt @ point_src + T_gt

    return point_src, point_target, R_gt


def main():
    point_src, point_target, R_gt = generate_simulation_data()
    print('point_src', point_src)

    print('icp (so3 + translation) start...')
    icp_so3_and_t.icp_so3_and_translation_numirical(point_src, point_target)

    print('icp se3 start...')
    icp_se3.icp_se3_numirical(point_src, point_target)

if __name__ == "__main__":
    main()
