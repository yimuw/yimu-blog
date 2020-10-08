import numpy as np
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


if __name__ == "__main__":
    linear_case()
