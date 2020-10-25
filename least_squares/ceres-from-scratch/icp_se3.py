from number_forward_flow import *


def euler_angle_to_rotation(yaw, pitch, roll):
    from math import cos, sin
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
    from math import cos, sin
    theta = np.linalg.norm(w)
    # Approximation when theta is small
    if(abs(theta) < 1e-8):
        return np.identity(3) + skew(w)

    normalized_w = w / theta
    K = skew(normalized_w)
    # Rodrigues
    R = np.identity(3) + sin(theta) * K + (1 - cos(theta)) * K @ K
    np.testing.assert_almost_equal(R @ R.transpose(), np.identity(3))

    return R

def V_operator(w):
    from math import cos, sin
    w_skew = skew(w)
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
    SE3_mat[:3, :3] = so3_exp(w)
    # I hate numpy
    SE3_mat[:3, 3] = (V_operator(w) @ t).flatten()

    return SE3_mat

def icp_se3():
    print('=============== icp_se3 ==============')

    T = np.identity(4)
    T[:3,:3] = euler_angle_to_rotation(0.2, 0.1, 0.3)
    T[:3, 3] = np.array([1., 2., 3.])

    src = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 1, 1],
        [1, 0, 1],
        [0, 2, -1],
    ])
    # homogenous coordinate
    src = np.hstack([src, np.ones([src.shape[0],1])]).T
    
    target = T @ src

    def icp_residual_i(se3_para, T, src_i, target_i):
        # this is the true function
        #  return exp(se3_para) @ T @ src_i - target_i
        # but need to implement a couple of operators for so3_exp
        wx,wy,wz,tx,ty,tz = se3_para
        SE3_approx = np.array([
            [1., -wz, wy, tx],
            [wz, 1., -wx, ty],
            [-wy, wx, 1., tz],
            [0., 0., 0., 1.],
        ])

        return (SE3_approx @ T @ src_i - target_i)[:3]

    T_var = np.identity(4)

    for iter in range(20):
        local_se3 = np.array([0., 0., 0., 0., 0., 0.]).T
        lhs = np.zeros([6, 6])
        rhs = np.zeros(6)

        cost = 0
        for i in range(src.shape[1]):
            src_i = src[:, i]
            target_i = target[:, i]
            r, J = ResidualBlock(lambda param: icp_residual_i(
            param, T_var, src_i, target_i), local_se3).evaluate()
            lhs += J.T @ J
            rhs -= J.T @ r
            cost += np.linalg.norm(r)
        print('iter', iter, 'cost:', cost)
        local_se3_delta = 0.8 * np.linalg.solve(lhs, rhs)
        T_var = se3_exp(local_se3_delta) @ T_var

    print('T_var:', T_var)
    print('T_gt: ', T)


if __name__ == "__main__":
    icp_se3()
