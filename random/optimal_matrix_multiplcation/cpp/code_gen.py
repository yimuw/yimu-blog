import timeit
import numpy as np
import inspect


class Node:
    def __init__(self, type):
        self.node_type = type
        self.shape = None
        self.expression_string = None


class OpNode(Node):
    def __init__(self, op):
        super().__init__('OpNode')
        self.left = None
        self.right = None
        self.op = op


class VarNode(Node):
    def __init__(self, var):
        super().__init__('VarNode')
        self.var = var
        self.shape = var.data.shape
        self.expression_string = var.name


class Variable():
    def __init__(self, name, data):
        self.name = name
        self.data = data


def print_tree(root):
    '''
        https://stackoverflow.com/questions/34012886/print-binary-tree-level-by-level-in-python
    '''
    def _display_aux(root):
        # No child.
        if root.node_type == 'VarNode':
            line = root.var.name
            width = len(line)
            height = 1
            middle = width // 2
            return [line], width, height, middle

        # Two children.
        left, n, p, x = _display_aux(root.left)
        right, m, q, y = _display_aux(root.right)
        s = root.op
        u = len(s)
        first_line = (x + 1) * ' ' + (n - x - 1) * \
            '_' + s + y * '_' + (m - y) * ' '
        second_line = x * ' ' + '/' + \
            (n - x - 1 + u + y) * ' ' + '\\' + (m - y - 1) * ' '
        if p < q:
            left += [n * ' '] * (q - p)
        elif q < p:
            right += [m * ' '] * (p - q)
        zipped_lines = zip(left, right)
        lines = [first_line, second_line] + \
            [a + u * ' ' + b for a, b in zipped_lines]
        return lines, n + m + u, max(p, q) + 2, n + u // 2
    
    lines, _, _, _ = _display_aux(root)
    for line in lines:
        print(line)

def build_all_expression_tree(vars):
    '''
        build all trees for a list of mats
    '''
    if len(vars) == 1:
        return [VarNode(vars[0])]

    trees = []

    for i in range(1, len(vars)):
        left_trees = build_all_expression_tree(vars[:i])
        right_trees = build_all_expression_tree(vars[i:])

        for l in left_trees:
            for r in right_trees:
                root = OpNode('Prod')
                root.left = l
                root.right = r
                root.expression_string = \
                    'Prod<{},{}>'.format(root.left.expression_string,
                                     root.right.expression_string)
                trees.append(root)
    return trees


def gen_test_data():
    M1 = Variable('M1', np.ones([20, 20]))
    M2 = Variable('M2', np.ones([20, 35]))
    M3 = Variable('M3', np.ones([35, 100]))
    M4 = Variable('M4', np.ones([100, 360]))
    M5 = Variable('M5', np.ones([100, 360]))
    vars = [M1, M2, M3, M4, M5]
    return vars

def max_opetation_for_expressions(expressions):
    if len(expressions) == 1:
        return expressions[0]

    e1 = expressions[0]
    s = 'Max<{},\n{}>'.format(e1, 
        max_opetation_for_expressions(expressions[1:]))
    return s


def test_all_tree():
    vars = gen_test_data()

    trees = build_all_expression_tree(vars)

    for t in trees:
        print('for tree:', t.expression_string)
        print_tree(t)

    expressions = [t.expression_string for t in trees]
    print('All expressions:')

    var_size = len(vars)
    for i, e in enumerate(expressions):
        print('using E{}{} = {};'.format(var_size, i, e))

    print('using OptimalExpression = {}::Type;'.format(max_opetation_for_expressions(expressions)))

if __name__ == "__main__":
    test_all_tree()

