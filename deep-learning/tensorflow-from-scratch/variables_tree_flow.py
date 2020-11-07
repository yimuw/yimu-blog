import math
from collections import defaultdict
import numbers
import numpy as np
import utils


class Node:
    def __init__(self, id):
        self.id = id
        # parent operators
        self.parents = []
        # child operator
        self.children = []


class Variable(Node):

    def __init__(self, value=0, id='', ntype='opt-varible'):
        super().__init__(id)

        self.value = value
        self.grad = 0

        # "opt-varible", "const", "intermediate"
        self.ntype = ntype

    def __add__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        plus = Plus(self, other)
        # GLOBAL_VARS.operators.append(plus)

        self.parents.append(plus)
        other.parents.append(plus)
        plus.result.children = [plus]
        return plus.result

    def __radd__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        return other + self

    def __sub__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        neg_other = - other
        return self + neg_other

    def __rsub__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        return other - self

    def __mul__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        mul = Mul(self, other)

        self.parents.append(mul)
        other.parents.append(mul)
        mul.result.children = [mul]
        return mul.result

    def __rmul__(self, other):
        if isinstance(other, numbers.Number):
            other = Variable(value=other, ntype='const')
        return other * self

    def __neg__(self):
        neg = Neg(self)

        self.parents.append(neg)
        neg.result.children = [neg]
        return neg.result

    def __str__(self):
        return 'value:{} grad:{} #children:{} #parents:{}'.format(self.value, self.grad, len(self.children),
                                                                  len(self.parents))


def ntf_sigmoid(number):
    if isinstance(number, Variable):
        sigmoid_operator = Sigmoid(number)
        number.parents.append(sigmoid_operator)
        sigmoid_operator.result.children = [sigmoid_operator]
        return sigmoid_operator.result
    else:
        if abs(number) > 50:
            number = np.sign(number) * 50
        expo = math.exp(number)
        #print('expo:', expo, self.a.value, expo / (1 + expo))
        return expo / (1 + expo)


def ntf_log(number):
    if isinstance(number, Variable):
        log_operator = Log(number)
        number.parents.append(log_operator)
        log_operator.result.children = [log_operator]
        return log_operator.result
    else:
        return math.log(number)


class Operator(Node):
    def __init__(self, id):
        super().__init__(id)

    def forward(self):
        raise NotImplementedError("Should have implemented this")

    def backward(self):
        raise NotImplementedError("Should have implemented this")


class Plus(Operator):
    def __init__(self, a: Variable, b: Variable):
        super().__init__("({})+({})".format(a.id, b.id))
        self.a = a
        self.b = b
        self.result = Variable(ntype="intermediate",
                               id="res:{}".format(self.id))

        self.children = [a, b]
        self.parents = [self.result]

    def forward(self):
        self.result.value = self.a.value + self.b.value

    def backward(self):
        if self.a is self.b:
            self.a.grad += 2 * self.result.grad
        else:
            self.a.grad += self.result.grad
            self.b.grad += self.result.grad


class Mul(Operator):
    def __init__(self, a: Variable, b: Variable):
        super().__init__("({})*({})".format(a.id, b.id))
        self.a = a
        self.b = b
        self.result = Variable(ntype="intermediate",
                               id="res:{}".format(self.id))

        self.children = [a, b]
        self.parents = [self.result]

    def forward(self):
        self.result.value = self.a.value * self.b.value

    def backward(self):
        if self.a is self.b:
            self.a.grad += 2 * self.result.grad * self.a.value
        else:
            self.a.grad += self.result.grad * self.b.value
            self.b.grad += self.result.grad * self.a.value


class Neg(Operator):
    def __init__(self, a: Variable):
        super().__init__("-({})".format(a.id))
        self.a = a
        self.result = Variable(ntype="intermediate",
                               id="res:{}".format(self.id))

        self.children = [a]
        self.parents = [self.result]

    def forward(self):
        self.result.value = - self.a.value

    def backward(self):
        self.a.grad += - self.result.grad


class Sigmoid(Operator):
    def __init__(self, a: Variable):
        super().__init__("sigmoid({})".format(a.id))
        self.a = a
        self.result = Variable(ntype="intermediate",
                               id="res:{}".format(self.id))

        self.children = [a]
        self.parents = [self.result]

    def forward(self):
        if abs(self.a.value) > 50:
            self.a.value = np.sign(self.a.value) * 50
        expo = math.exp(self.a.value)
        #print('expo:', expo, self.a.value, expo / (1 + expo))
        self.result.value = expo / (1 + expo)

    def backward(self):
        expo = math.exp(self.a.value)
        sigmoid = expo / (1 + expo)
        sigmoid_grad = sigmoid * (1 - sigmoid)
        self.a.grad += self.result.grad * sigmoid_grad


class Log(Operator):
    def __init__(self, a: Variable):
        super().__init__("log({})".format(a.id))
        self.a = a
        self.result = Variable(ntype="intermediate",
                               id="res:{}".format(self.id))

        self.children = [a]
        self.parents = [self.result]

    def forward(self):
        # utils.traverse_tree(self)
        self.result.value = math.log(self.a.value)

    def backward(self):
        self.a.grad += self.result.grad * (1. / self.a.value)


class NumberFlowCore:
    def __init__(self, cost_node, method='iter'):

        self.cost_node = cost_node
        self.all_nodes, self.varible_nodes, self.const_nodes = self.__get_all_nodes(
            cost_node)

        # remove duplication
        self.all_nodes = list(set(self.all_nodes))
        self.varible_nodes = list(set(self.varible_nodes))
        self.const_nodes = list(set(self.const_nodes))

        self.method = method
        if self.method == 'iter':
            self.__topological_sort()

    def __get_all_nodes(self, node):
        allnodes = [node]
        all_leaf_nodes = [node] if (isinstance(
            node, Variable) and node.ntype == 'opt-varible') else []
        all_const_nodes = [node] if (isinstance(
            node, Variable) and node.ntype == 'const') else []

        for c in node.children:
            sub_allnodes, sub_all_leaf_nodes, sub_all_const_nodes = self.__get_all_nodes(
                c)
            allnodes += sub_allnodes
            all_leaf_nodes += sub_all_leaf_nodes
            all_const_nodes += sub_all_const_nodes

        return allnodes, all_leaf_nodes, all_const_nodes

    def __topological_sort(self):
        zero_degree_nodes = []
        indegree = defaultdict(int)
        for node in self.all_nodes:
            indegree[node] = len(node.children)
            if len(node.children) == 0:
                zero_degree_nodes.append(node)

        topo_order = []
        while zero_degree_nodes:
            next_zero_degree_nodes = []
            for znode in zero_degree_nodes:
                topo_order.append(znode)
                for p in znode.parents:
                    indegree[p] -= 1
                    if indegree[p] == 0:
                        next_zero_degree_nodes.append(p)

            zero_degree_nodes = next_zero_degree_nodes

        if len(topo_order) != len(self.all_nodes):
            raise RuntimeError("cycle found!")

        self.topologic_order = topo_order

    def forward(self):
        if self.method == 'iter':
            self.__forward_iterative()
        elif self.method == 'recur':
            self.__forward_recursive()

    def __forward_iterative(self):
        for node in self.topologic_order:
            if isinstance(node, Operator):
                node.forward()

    def __forward_recursive(self):

        states = {}

        def evaluate(node):
            if node in states:
                if states[node] == 'visiting':
                    raise RuntimeError("cycle found!")
                else:
                    return states[node]

            states[node] = 'visiting'

            for child in node.children:
                evaluate(child)

            if isinstance(node, Operator):
                node.forward()
            if isinstance(node, Variable):
                states[node] = node.value

        evaluate(self.cost_node)

    def backward(self):
        if self.method == 'recur':
            return self.__backward_recur()
        elif self.method == 'iter':
            return self.__backward_iter()

    def __backward_recur(self):
        visited = set()

        def backward_dfs(node):
            if node in visited: return
            
            if isinstance(node, Operator):
                node.backward()
                visited.add(node)

            for child in node.children:
                backward_dfs(child)

        self.cost_node.grad = 1.
        backward_dfs(self.cost_node)

    def __backward_iter(self):
        self.cost_node.grad = 1.
        for node in reversed(self.topologic_order):
            if isinstance(node, Operator):
                node.backward()

    def clear_grad(self):
        def clear_grad_dfs(node):
            if isinstance(node, Variable):
                node.grad = 0
            for child in node.children:
                clear_grad_dfs(child)

        clear_grad_dfs(self.cost_node)

    def gradient_desent(self, rate=0.01):
        for var in self.varible_nodes:
            var.value -= rate * var.grad
