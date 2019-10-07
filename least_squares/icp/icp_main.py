import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt

import icp_euler
import icp_so3

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
    R_gt = icp_euler.euler_angle_to_rotation(0.84, pi, pi)
    point_target = R_gt @ point_src

    return point_src, point_target, R_gt


def main():
    point_src, point_target, R_gt = generate_simulation_data()
    print('point_src', point_src)

    print('icp euler start...')
    icp_euler.icp_yaw_pitch_roll_numirical(point_src, point_target)

    print('icp so3 start...')
    icp_so3.icp_so3_numirical(point_src, point_target)

    print('icp local so3 start...')
    icp_so3.icp_local_so3_numirical(point_src, point_target)

if __name__ == "__main__":
    main()
