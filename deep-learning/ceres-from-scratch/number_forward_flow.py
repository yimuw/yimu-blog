import numpy as np
from collections import defaultdict


class GlobalVars:
    def __init__(self):
        self.operators = []
        self.number_idx = 0
        self.plus_idx = 0
        self.mul_idx = 0


GLOBAL_VARS = GlobalVars()


class JetMapImpl:
    def __init__(self, value, var_id = None):
        self.value = value
        self.perturb = defaultdict(float)
        self.var_id = var_id
        if var_id is not None:
            self.perturb = {var_id: 1.}

    def __repr__(self):
        s = 'value:' + str(self.value) + '\n'
        for p in self.perturb:
            s += 'id:'+ str(p) + ' :' + str(self.perturb[p]) + '\n'
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
            other = JetMapImpl(other.value)

        ret = JetMapImpl(value=self.value - other.value)
        ret.perturb = defaultdict(float)

        for var in self.perturb:
            ret.perturb[var] -= self.perturb[var]
        for var in other.perturb:
            ret.perturb[var] -= other.perturb[var]
        return ret


class ResidualBlock:
    def __init__(self, residual_function, init_vars):
        self.residual_function = residual_function
        self.jets = [JetMapImpl(v, var_id='var{}'.format(i)) for i,v in enumerate(init_vars)]


    def evaluate(self):
        jet_residual = self.residual_function(self.jets)
        
        jacobian = np.zeros([len(jet_residual), len(jets)])
        for ridx in range(len(jet_residual)):
            for vidx in range(len(jets)):
                jacobian[ridx, vidx] = jet_residual[ridx].perturb[jets[vidx].var_id]

        return jacobian



if __name__ == "__main__":
    A = np.random.rand(10,5)
    x = np.ones(5,dtype='float64')



    b = A @ x


    jets = [JetMapImpl(v, var_id='var{}'.format(i)) for i,v in enumerate(x)]

    def residual_test(vars):
        ret = []
        for row in A:
            prod = 0.
            for i, a in enumerate(row):
                prod = vars[i] * a + prod
            ret.append(prod)
        
        return ret

    r = ResidualBlock(residual_test, x)
    J = r.evaluate()
    print('J:', J)
    print('A:', A)
