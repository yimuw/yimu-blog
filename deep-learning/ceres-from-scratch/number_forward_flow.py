import numpy as np
import math
from collections import defaultdict


class JetMapImpl:
    def __init__(self, value, var_id=None):
        self.value = value
        self.perturb = defaultdict(float)
        self.var_id = var_id
        if var_id is not None:
            self.perturb = {var_id: 1.}

    def __repr__(self):
        s = 'value:' + str(self.value) + '\n'
        for p in self.perturb:
            s += 'id:' + str(p) + ' :' + str(self.perturb[p]) + '\n'
        return s

    def __add__(self, other):
        if isinstance(other, float):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value + other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var]
        for var in other.perturb:
            ret.perturb[var] += other.perturb[var]
        return ret

    def __mul__(self, other):
        if isinstance(other, float):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value * other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var] * other.value

        for var in other.perturb:
            ret.perturb[var] += self.value * other.perturb[var]
        return ret

    def __sub__(self, other):
        if isinstance(other, float):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value - other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] += self.perturb[var]
        for var in other.perturb:
            ret.perturb[var] -= other.perturb[var]
        return ret


def cosine(num):
    if isinstance(num, float):
        return math.cos(num)
    else:
        ret = JetMapImpl(value=math.cos(num.value))
        ret.perturb = defaultdict(float)

        for var in num.perturb:
            ret.perturb[var] += math.cos(num.perturb[var])

        return ret


def sine(num):
    if isinstance(num, float):
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
        self.jets = [JetMapImpl(v, var_id='var{}'.format(i))
                     for i, v in enumerate(init_vars)]

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
        ret = []

        rows = len(A)
        cols = len(A[0])

        for r in range(rows):
            prod = 0.
            for c in range(cols):
                prod = vars[c] * A[r, c] + prod
            ret.append(prod - b[r])

        return ret

    x0 = np.random.rand(5, 1)
    r = ResidualBlock(residual_test, x0)
    r, J = r.evaluate()
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


if __name__ == "__main__":
    linear_case()
    motion_case()
