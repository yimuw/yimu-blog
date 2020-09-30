import math
from collections import defaultdict


class GlobalVars:
    def __init__(self):
        self.operators = []
        self.number_idx = 0
        self.plus_idx = 0
        self.mul_idx = 0


GLOBAL_VARS = GlobalVars()


class Node:
    def __init__(self, id):
        self.id = id
        # parent operators
        self.parents = []
        # child operator
        self.children = []


class Number(Node):

    def __init__(self, value=0, id="", ntype="varible"):
        super().__init__(id)

        self.value = value
        self.grad = 0

        # "varible", "const", "intermediate"
        self.ntype = ntype

    def __add__(self, other):
        plus = Plus(self, other)
        GLOBAL_VARS.operators.append(plus)

        self.parents.append(plus)
        other.parents.append(plus)
        plus.result.children = [plus]
        return plus.result

    def __mul__(self, other):
        mul = Mul(self, other)
        GLOBAL_VARS.operators.append(mul)

        self.parents.append(mul)
        other.parents.append(mul)
        mul.result.children = [mul]
        return mul.result

    def __str__(self):
        return 'value:{} grad:{} #children:{} #parents:{}'.format(self.value, self.grad, len(self.parents),
                                                                  len(self.parents))


class Operator(Node):
    def __init__(self, id):
        super().__init__(id)

    def forward(self):
        raise NotImplementedError("Should have implemented this")

    def backward(self):
        raise NotImplementedError("Should have implemented this")


class Plus(Operator):
    def __init__(self, a: Number, b: Number):
        super().__init__("({})+({})".format(a.id, b.id))
        self.a = a
        self.b = b
        self.result = Number(ntype="intermediate", id="res:{}".format(self.id))

        self.children = [a, b]
        self.parents = [self.result]

    def forward(self):
        self.result.value = self.a.value + self.b.value

    def backward(self):
        self.a.grad = self.result.grad
        self.b.grad = self.result.grad


class Mul(Operator):
    def __init__(self, a: Number, b: Number):
        super().__init__("({})+({})".format(a.id, b.id))
        self.a = a
        self.b = b
        self.result = Number(ntype="intermediate", id="res:{}".format(self.id))

        self.children = [a, b]
        self.parents = [self.result]

    def forward(self):
        self.result.value = self.a.value * self.b.value

    def backward(self):
        self.a.grad = self.result.grad * self.b.value
        self.b.grad = self.result.grad * self.a.value


class NumberFlowCore:
    def __init__(self):
        self.topologic_order = []

    def topological_sort(self, cost_node):

        def get_all_nodes(node):
            ret = [node]
            for c in node.children:
                ret += get_all_nodes(c)
            return ret

        all_nodes = get_all_nodes(cost_node)

        zero_degree_nodes = []
        indegree = defaultdict(int)
        for node in all_nodes:
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

        #print("topo_order:", topo_order)
        #print("all_nodes:", all_nodes)
        if len(topo_order) != len(all_nodes):
            raise RuntimeError("cycle found!")

        self.topologic_order = topo_order

    def forward(self, cost_node):
        self.topological_sort(cost_node)

        for node in self.topologic_order:
            if isinstance(node, Number):
                continue
            node.forward()
