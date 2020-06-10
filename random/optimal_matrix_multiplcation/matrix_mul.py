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



def count_num_muls(trees):
    dp_map = {}

    def count_num_muls_internal(root):
        s = root.expression_string
        if s in dp_map:
            count, shape = dp_map[s]
            # update the shape of mul
            root.shape = shape
            return count

        if root.node_type == 'VarNode':
            return 0

        assert root.node_type == 'OpNode'
        left_muls = count_num_muls_internal(root.left)
        right_muls = count_num_muls_internal(root.right)

        lr, lc = root.left.shape
        rr, rc = root.right.shape
        assert lc == rr, 'matrix dim mismatch'
        root.shape = (lr, rc)
        cur_muls = lc * lr * rc

        total_muls = left_muls + right_muls + cur_muls
        # need to track the shape of mul
        dp_map[s] = (total_muls, root.shape)
        return total_muls

    for t in trees:
        yield count_num_muls_internal(t)


class Variable():
    def __init__(self, name, data):
        self.name = name
        self.data = data


def build_expression_tree_simple(vars):
    '''
        build a single tree for a list of mats
    '''
    if len(vars) == 1:
        return VarNode(vars[0])
    op_node = OpNode('@')
    op_node.left = VarNode(vars[0])
    op_node.right = build_expression_tree_simple(vars[1:])
    op_node.expression_string = \
        '({}*{})'.format(op_node.left.expression_string,
                         op_node.right.expression_string)
    return op_node

def build_expression_tree_mid(vars):
    '''
        build a single tree for a list of mats
    '''
    if len(vars) == 1:
        return VarNode(vars[0])
    op_node = OpNode('@')
    var_len = len(vars)
    op_node.left = build_expression_tree_simple(vars[: var_len // 2])
    op_node.right = build_expression_tree_simple(vars[var_len // 2 :])
    op_node.expression_string = \
        '({}*{})'.format(op_node.left.expression_string,
                         op_node.right.expression_string)
    return op_node


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
                root = OpNode('@')
                root.left = l
                root.right = r
                root.expression_string = \
                    '({}@{})'.format(root.left.expression_string,
                                     root.right.expression_string)
                trees.append(root)
    return trees


def gen_test_data():
    A = Variable('A', np.ones([20, 20]))
    B = Variable('B', np.ones([20, 35]))
    C = Variable('C', np.ones([35, 100]))
    D = Variable('D', np.ones([100, 360]))
    E = Variable('E', np.ones([360, 10]))
    F = Variable('F', np.ones([10, 10]))
    vars = [A, B, C, D, E, F]
    return vars

def test_single_tree():
    vars = gen_test_data()
    tree = build_expression_tree_simple(vars)
    print('for tree:', tree.expression_string)
    print_tree(tree)

    tree = build_expression_tree_mid(vars)
    print('for tree:', tree.expression_string)
    print_tree(tree)


def test_all_tree():
    vars = gen_test_data()

    trees = build_all_expression_tree(vars)

    muls = list(count_num_muls(trees))
    for t, m in zip(trees, muls):
        print('for tree:', t.expression_string)
        print_tree(t)
        print('# muls:', m)
        print('')

    min_tree, min_muls = min(zip(trees, muls), key=lambda x: x[1])
    print('optimal expression:', min_tree.expression_string)
    print_tree(min_tree)
    print('optimal # muls:', min_muls)


def time_expr():
    print('timming....')
    setup_code = ''' 
from __main__ import gen_test_data
vars = gen_test_data()
data = [v.data for v in vars]
A, B, C, D, E, F = data
'''

    test_code1 = ''' 
val = (A@(B@(C@(D@(E@F)))))
    '''

    test_code2 = ''' 
val = A@B@C@D@E@F
    '''

    test_code3 = '''
val = (((((A@B)@C)@D)@E)@F)
    '''

    test_code_opt = ''' 
val = (A@((B@(C@(D@E)))@F))
    '''
    times = timeit.repeat(setup=setup_code,
                          stmt=test_code1,
                          number=10000)
    print('time for {} is {} ms'.format(test_code1, min(times)))
    
    times = timeit.repeat(setup=setup_code,
                          stmt=test_code2,
                          number=10000)
    print('time for {} is {} ms'.format(test_code2, min(times)))

    times = timeit.repeat(setup=setup_code,
                          stmt=test_code3,
                          number=10000)
    print('time for {} is {} ms'.format(test_code3, min(times)))

    times = timeit.repeat(setup=setup_code,
                          stmt=test_code_opt,
                          number=10000)
    print('time for {} is {} ms'.format(test_code_opt, min(times)))



if __name__ == "__main__":
    test_all_tree()
 
    time_expr()
