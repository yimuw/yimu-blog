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


def icp_so3():
    APPLY_CAUCHY_LOSS = True
    ADD_WRONG_ASSOCIATION = True

    print('=============== icp_so3 ==============')
    R = euler_angle_to_rotation(0.2, 0.1, 0.3)
    assert(abs(np.linalg.det(R) - 1) < 1e-4)
    t = np.array([1., 2., 3.]).T

    src = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 1, 1],
        [1, 0, 1],
        [0, 2, -1],
    ])

    def transform(p, R, t):
        return R @ p + t

    def transform_all(src, R, t):
        return (R @ src.T + t[:, np.newaxis]).T

    target = transform_all(src, R, t)

    if ADD_WRONG_ASSOCIATION:
        src = np.vstack([src, np.array([1, 2, 3])])
        target = np.vstack([target, np.array([10, 20, 30])])

    def icp_residual_i(epsilonWithT, Rot, src_i, target_i):
        E = skew(epsilonWithT[:3])
        t = epsilonWithT[3:]

        # this is the true function
        #  return transform(src_i, so3_exp(epsilonWithT[:3]) @ R, t) - target_i
        # but need to implement a couple of operators for so3_exp
        return transform(src_i, (E + np.identity(3)) @ Rot, t) - target_i

    epsilonWithT_var = np.array([0., 0., 0., 0., 0., 0.]).T
    R_var = np.identity(3)


    for iter in range(20):
        lhs = np.zeros([6, 6])
        rhs = np.zeros(6)
        cost = 0
        for src_i, target_i in zip(src, target):
            r, J = ResidualBlock(lambda param: icp_residual_i(
                param, R_var, src_i, target_i), epsilonWithT_var).evaluate()
            if APPLY_CAUCHY_LOSS:
                def cauchy_dot(s):
                    return 1 / (1 + s)
                cauchy_weigth = cauchy_dot(r.T @ r)
                lhs += cauchy_weigth * J.T @ J
                rhs -= cauchy_weigth * J.T @ r
                cost += math.log(1 + np.linalg.norm(r))
            else:
                lhs += J.T @ J
                rhs -= J.T @ r
                cost += np.linalg.norm(r)
        print('iter:', iter, 'cost:', cost)
        delta = 0.8 * np.linalg.solve(lhs, rhs)
        R_var = so3_exp(delta[:3]) @ R_var
        epsilonWithT_var[3:] += delta[3:]
    print('t_est:', epsilonWithT_var[3:])
    print('t_gt: ', t)
    print('R_var:', R_var)
    print('R_gt: ', R)


if __name__ == "__main__":
    icp_so3()
