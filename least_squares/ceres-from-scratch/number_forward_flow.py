import numpy as np
import math
from collections import defaultdict
import itertools
import numbers


class JetMapImpl:
    def __init__(self, value, var_id=None):
        self.value = value
        self.derivative_wrt_variables = defaultdict(float)
        self.var_id = var_id
        if var_id is not None:
            self.derivative_wrt_variables = {var_id: 1.}

    def __repr__(self):
        s = 'value:' + str(self.value) + ' '
        for p in self.derivative_wrt_variables:
            s += 'id:' + str(p) + ' :' + str(self.derivative_wrt_variables[p]) + ' '
        return s

    def __add__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value + other.value)
        ret.derivative_wrt_variables = defaultdict(float)

        for var in self.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += self.derivative_wrt_variables[var]
        for var in other.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += other.derivative_wrt_variables[var]
        return ret

    def __radd__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        return other + self

    def __mul__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value * other.value)
        ret.derivative_wrt_variables = defaultdict(float)

        for var in self.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += self.derivative_wrt_variables[var] * other.value

        for var in other.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += self.value * other.derivative_wrt_variables[var]
        return ret

    def __rmul__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        return other * self

    def __sub__(self, other):
        if isinstance(other, numbers.Number):
            other = JetMapImpl(other)

        ret = JetMapImpl(value=self.value - other.value)
        ret.derivative_wrt_variables = defaultdict(float)

        for var in self.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += self.derivative_wrt_variables[var]
        for var in other.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] -= other.derivative_wrt_variables[var]
        return ret

    def __neg__(self):
        ret = JetMapImpl(value=-self.value)
        for var in self.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] = - self.derivative_wrt_variables[var]
        return ret


def cosine(num):
    if isinstance(num, numbers.Number):
        return math.cos(num)
    else:
        ret = JetMapImpl(value=math.cos(num.value))
        ret.derivative_wrt_variables = defaultdict(float)

        for var in num.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += - math.sin(num.value) * num.derivative_wrt_variables[var]

        return ret


def sine(num):
    if isinstance(num, numbers.Number):
        return math.sin(num)
    else:
        ret = JetMapImpl(value=math.sin(num.value))
        ret.derivative_wrt_variables = defaultdict(float)

        for var in num.derivative_wrt_variables:
            ret.derivative_wrt_variables[var] += math.cos(num.value) * num.derivative_wrt_variables[var]

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
                jacobian[ridx, vidx] = jet_residual[ridx].derivative_wrt_variables[self.jets[vidx].var_id]
        residual = np.array([j.value for j in jet_residual])
        return residual, jacobian
