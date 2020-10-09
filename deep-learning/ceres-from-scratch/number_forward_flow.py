import numpy as np
import math
from collections import defaultdict
import itertools
import numbers


class JetMapImpl:
    def __init__(self, value, var_id=None):
        self.value = value
        self.perturb = defaultdict(float)
        self.var_id = var_id
        if var_id is not None:
            self.perturb = {var_id: 1.}

    def __repr__(self):
        s = 'value:' + str(self.value) + ' '
        for p in self.perturb:
            s += 'id:' + str(p) + ' :' + str(self.perturb[p]) + ' '
        return s

    def __add__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value + other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var]
        for var in other.perturb:
            ret.perturb[var] += other.perturb[var]
        return ret

    def __radd__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        return other + self

    def __mul__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value * other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var] * other.value

        for var in other.perturb:
            ret.perturb[var] += self.value * other.perturb[var]
        return ret

    def __rmul__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        return other * self

    def __sub__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value - other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var]
        for var in other.perturb:
            ret.perturb[var] -= other.perturb[var]
        return ret

    def __neg__(self):
        ret = JetMapImpl(value=-self.value)
        for var in self.perturb:
            ret.perturb[var] = - self.perturb[var]
        return ret

def cosine(num):
    if isinstance(num, numbers.Number):
        return math.cos(num)
    else:
        ret = JetMapImpl(value=math.cos(num.value))
        ret.perturb = defaultdict(float)

        for var in num.perturb:
            ret.perturb[var] += math.cos(num.perturb[var])

        return ret


def sine(num):
    if isinstance(num, numbers.Number):
        return math.sin(num)
    else:
        ret = JetMapImpl(value=math.sin(num.value))
        ret.perturb = defaultdict(float)

        for var in num.perturb:
            ret.perturb[var] += math.sin(num.perturb[var])

        return ret


class ResidualBlock:
    def __init__(self, residual_function, init_vars):
        self.residual_function = residual_function
        self.jets = np.array([JetMapImpl(v, var_id='var{}'.format(i))
                     for i, v in enumerate(init_vars)])

    def evaluate(self):
        jet_residual = self.residual_function(self.jets)

        jacobian = np.zeros([len(jet_residual), len(self.jets)])
        for ridx in range(len(jet_residual)):
            for vidx in range(len(self.jets)):
                jacobian[ridx, vidx] = jet_residual[ridx].perturb[self.jets[vidx].var_id]
        residual = [j.value for j in jet_residual]
        return residual, jacobian


def linear_case():
    print('=============== linear_case ==============')
    A = np.random.rand(6, 5)
    x_gt = np.ones(5, dtype='float64')

    b = A @ x_gt

    def residual_test(vars):
        """
        r = Ax - b
        """
        ret = A @ vars - b
        return ret

    x0 = np.random.rand(5, 1)
    r, J = ResidualBlock(residual_test, x0).evaluate()
    print('r:', r)
    print('J:', J)
    print('A:', A)

    J = np.array(J)
    r = np.array(r)
    dx = np.linalg.solve(J.T @ J, -J.T @ r)
    print('x0:', x0.T)
    print('dx:', dx.T)
    print('solver res:', (x0 + dx).T)
    print('x_gt:', x_gt.T)


def motion_case():
    print('=============== motion (nonlinear) case ==============')

    def motion_func(state):
        x, y, v, theta = state

        xnext = x + cosine(theta) * v
        ynext = y + sine(theta) * v

        return [xnext, ynext, v, theta]

    def observation_func(state):
        x, y, v, theta = state
        return x, y

    s0 = [0., 0., 0.5, math.pi / 9]
    s1 = motion_func(s0)

    o0 = observation_func(s0)
    o1 = observation_func(s1)

    v0 = [0., 0., 1., math.pi / 4]
    v1 = [0., 0., 0., 0.]
    states = v0 + v1

    def motion_residaul(state0andstate1):
        state0 = state0andstate1[:4]
        state1 = state0andstate1[4:]
        pred = motion_func(state0)
        return [b - a for a, b in zip(pred, state1)]

    def observation_residual0(state0andstate1):
        state0 = state0andstate1[:4]
        pred = observation_func(state0)
        return [a - b for a, b in zip(pred, o0)]

    def observation_residual1(state0andstate1):
        state1 = state0andstate1[4:]
        pred = observation_func(state1)
        return [a - b for a, b in zip(pred, o1)]

    for iter in range(10):
        full_jacobian = np.zeros([0, 8])
        residaul = np.zeros([0])

        r, J = ResidualBlock(observation_residual0, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        r, J = ResidualBlock(observation_residual1, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        r, J = ResidualBlock(motion_residaul, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        delta = np.linalg.solve(
            full_jacobian.T @ full_jacobian, - full_jacobian.T @ residaul)
        states = [s + 0.8 * d for s, d in zip(states, delta)]
        print('cost:', residaul.T @ residaul)

    print('result s0:', states[:4])
    print('gt     s0:', s0)
    print('result s1:', states[4:])
    print('gt     s1:', s1)


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

    normalized_w  = w / theta
    K = skew(normalized_w)
    # Rodrigues
    R = np.identity(3) + sin(theta) * K + (1 - cos(theta)) * K @ K
    np.testing.assert_almost_equal(R @ R.transpose(), np.identity(3))
    
    return R

def icp_so3():
    print('=============== icp_so3 ==============')
    R = euler_angle_to_rotation(0.2,0.1,0.3)
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
    print('target:', target)

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
        for src_i, target_i in zip(src, target):
            r, J = ResidualBlock(lambda param: icp_residual_i(param, R_var, src_i, target_i), epsilonWithT_var).evaluate()
            lhs += J.T @ J
            rhs -= J.T @ r
        delta = 0.8 * np.linalg.solve(lhs, rhs)
        R_var = so3_exp(delta[:3]) @ R_var
        epsilonWithT_var[3:] += delta[3:]
    print('t_est:', epsilonWithT_var[3:])
    print('R_var:', R_var)
    print('R_gt: ',R)


if __name__ == "__main__":
    linear_case()
    motion_case()
    icp_so3()
