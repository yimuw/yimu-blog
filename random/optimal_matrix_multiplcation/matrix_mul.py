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


class ExpressionTreeAlgo:
    def traverse(self, root):
        if root.node_type == 'VarNode':
            print('var_id:', root.var.name)
            return

        assert root.node_type == 'OpNode'
        self.traverse(root.left)
        print('op:', root.op)
        self.traverse(root.right)

    def count_num_muls(self, trees):
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

    var_node0 = VarNode(vars[0])

    op_node = OpNode('@')
    op_node.left = var_node0
    op_node.right = build_expression_tree_simple(vars[1:])
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
    v0 = Variable('v0', np.ones([20, 20]))
    v1 = Variable('v1', np.ones([20, 35]))
    v2 = Variable('v2', np.ones([35, 100]))
    v3 = Variable('v3', np.ones([100, 360]))
    v4 = Variable('v4', np.ones([360, 10]))
    v5 = Variable('v5', np.ones([10, 10]))
    vars = [v0, v1, v2, v3, v4, v5]
    return vars


def test_all_tree():
    vars = gen_test_data()

    trees = build_all_expression_tree(vars)
    algo = ExpressionTreeAlgo()

    muls = list(algo.count_num_muls(trees))
    for t, m in zip(trees, muls):
        print('for tree:', t.expression_string)
        print('# muls:', m)

    min_tree, min_muls = min(zip(trees, muls), key=lambda x: x[1])
    print('optimal expression:', min_tree.expression_string)
    print('optimal # muls:', min_muls)


def time_expr():
    print('timming....')
    SETUP_CODE = ''' 
from __main__ import gen_test_data
vars = gen_test_data()
data = [v.data for v in vars]
v0, v1, v2, v3, v4, v5 = data
'''

    TEST_CODE1 = ''' 
val = v0@v1@v2@v3@v4@v5
    '''

    TEST_CODE2 = ''' 
val = (v0@((v1@(v2@(v3@v4)))@v5))
    '''
    times = timeit.repeat(setup=SETUP_CODE,
                          stmt=TEST_CODE1,
                          number=10000)
    print('time for {}\n: {}'.format(TEST_CODE1, min(times)))
    times = timeit.repeat(setup=SETUP_CODE,
                          stmt=TEST_CODE2,
                          number=10000)
    print('time for {}\n: {}'.format(TEST_CODE2,min(times)))


if __name__ == "__main__":
    test_all_tree()

    time_expr()
