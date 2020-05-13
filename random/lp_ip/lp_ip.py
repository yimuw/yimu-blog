# Import packages.
import cvxpy as cp
import numpy as np
import random
from collections import defaultdict


class AssociationGraph:
    def __init__(self):
        self.layer_sizes = None
        # random experiment
        RANDOM_SIZE = True
        if RANDOM_SIZE:
            size = random.randint(2, 10)
            self.layer_sizes = [random.randint(2, 30) for i in range(size)]
        else:
            self.layer_sizes = np.array([3, 4, 3, 4])

        self.len_layers = len(self.layer_sizes)

        # association reward
        self.weights = []
        # map edges in the graph to a linear array
        self.weights_linear_map = [0]
        for i in range(self.len_layers - 1):
            # noise for association reward
            w = np.random.uniform(
                0.0, 0.1, (self.layer_sizes[i], self.layer_sizes[i+1]))
            # Associated nodes.
            # Without loss of generality, put associated cost on diag.
            min_len = min(self.layer_sizes[i], self.layer_sizes[i+1])
            diag_w = np.random.uniform(0.5, 1., min_len)
            # TODO: function? diag?
            for j in range(min_len):
                w[j][j] = diag_w[j]

            self.weights.append(w)
            self.weights_linear_map.append(w.size)

        self.start_reward = np.random.uniform(0.2, 0.2)
        self.end_reward = np.random.uniform(0.2, 0.2)

        self.weights_linear_map = np.cumsum(self.weights_linear_map)

    # l1<-layer       l2
    # ..u.           ..v.
    #
    def edge_map(self, layer, u_idx, v_idx):
        offset = self.weights_linear_map[layer]
        adj_mat_size = self.weights[layer].shape
        rows, cols = adj_mat_size

        return offset + u_idx * cols + v_idx

    # map the start edge for a node in a layer to a linear idx
    def start_edge_map(self, layer, u_idx):
        start_idx = self.weights_linear_map[-1] + sum(self.layer_sizes[:layer])
        result = start_idx + u_idx
        return result

    # map the end edge for a node in a layer to a linear idx
    def end_edge_map(self, layer, u_idx):
        start_idx = self.weights_linear_map[-1] + \
            sum(self.layer_sizes) + sum(self.layer_sizes[:layer])
        result = start_idx + u_idx
        return result

    def constraint_lp(self):
        num_vars = 0
        # a variable for a association
        for i in range(self.len_layers - 1):
            num_vars += self.layer_sizes[i] * self.layer_sizes[i+1]
        # a variable for the creation & the ending for each node
        num_vars += 2 * sum(self.layer_sizes)

        # num of constraint, 2 for each graph node
        A = np.zeros((2 * sum(self.layer_sizes), num_vars))

        print('num layers:', self.len_layers, ' num nodes:', sum(self.layer_sizes), 'num edges: ',
              num_vars, ' num graph constraint:', A.shape[0])

        constraint_idx = 0

        c = np.zeros(num_vars)

        for u_layer in range(len(self.layer_sizes) - 1):
            w = self.weights[u_layer]
            rows, cols = w.shape

            # update c for each edge
            for u in range(rows):
                for v in range(cols):
                    idx = self.edge_map(u_layer, u, v)
                    c[idx] = w[u][v]

            # constraints for out going edges
            for u in range(rows):
                for v in range(cols):
                    idx = self.edge_map(u_layer, u, v)
                    A[constraint_idx][idx] = 1.

                end_edge_idx = self.end_edge_map(u_layer, u)
                A[constraint_idx][end_edge_idx] = 1.
                constraint_idx += 1

            # constraints for in going edges
            for v in range(cols):
                for u in range(rows):
                    idx = self.edge_map(u_layer, u, v)
                    A[constraint_idx][idx] = 1.

                start_edge_idx = self.start_edge_map(u_layer + 1, v)
                A[constraint_idx][start_edge_idx] = 1.
                constraint_idx += 1

        # update c for start & end
        for layer in range(self.len_layers):
            layer_size = self.layer_sizes[layer]
            for u in range(layer_size):
                start_edge_idx = self.start_edge_map(layer, u)
                end_edge_idx = self.end_edge_map(layer, u)
                c[start_edge_idx] = self.start_reward
                c[end_edge_idx] = self.end_reward

        # start edges for the first layer
        for u in range(self.layer_sizes[0]):
            start_edge_idx = self.start_edge_map(0, u)
            A[constraint_idx][start_edge_idx] = 1.
            constraint_idx += 1

        # end edges for the last layer
        for u in range(self.layer_sizes[-1]):
            end_edge_idx = self.end_edge_map(len(self.layer_sizes) - 1, u)
            A[constraint_idx][end_edge_idx] = 1.
            constraint_idx += 1

        assert(constraint_idx == A.shape[0])
        b = np.ones(constraint_idx)

        if False:
            print('A:\n', A)

        # TODO: check if every row of A is not perpendicular to c?
        return A, b, c, num_vars

    def lp(self):
        A, b, c, num_vars = self.constraint_lp()

        # Define and solve the CVXPY problem.
        x = cp.Variable(num_vars)
        prob = cp.Problem(cp.Minimize(- c.T@x),
                          [A @ x <= b, x >= 0, x <= 1])
        prob.solve(solver=cp.OSQP, verbose=False, eps_abs=1e-8, eps_rel=1e-8)

        # Print result.
        if False:
            print("\nThe optimal value is", prob.value)
            print("A solution x is")
            print(x.value)

        # TODO: what's the precision for cvxpy?
        epsilon = 1e-7
        integral_mask = (np.abs(x.value-0) <
                         epsilon) | (np.abs(x.value-1) < epsilon)
        num_integral = sum(integral_mask)
        print("Num integral:", num_integral)
        relaxation_success = False
        if num_integral == num_vars:
            print("Integer Programming == Linear Programming !!")
            relaxation_success = True
        else:
            print("Linear relaxation != Integer Programming")
            relaxation_success = False

        if True and relaxation_success is False:
            for i in range(len(integral_mask)):
                if not integral_mask[i]:
                    print('i:', i, '  v:', x.value[i])

        return x.value, relaxation_success

    def build_graph(self, lp_result_x):
        # adjacent list
        direct_graph = defaultdict(list)

        # build the layer to layer graph
        for u_layer in range(len(self.layer_sizes) - 1):
            w = self.weights[u_layer]
            rows, cols = w.shape

            for v in range(cols):
                v_name = 'layer:{} node:{}'.format(u_layer + 1, v)
                for u in range(rows):
                    u_name = 'layer:{} node:{}'.format(u_layer, u)
                    x_idx = self.edge_map(u_layer, u, v)
                    direct_graph[v_name].append((u_name, lp_result_x[x_idx]))

        # connect to start
        start_name = 'start'
        for layer in range(self.len_layers):
            layer_size = self.layer_sizes[layer]
            for u in range(layer_size):
                u_name = 'layer:{} node:{}'.format(layer, u)
                x_idx = self.start_edge_map(layer, u)
                direct_graph[u_name].append((start_name, lp_result_x[x_idx]))
        direct_graph['start'] = [(None, 0.)]

        if False:
            for k, v in direct_graph.items():
                print('key: ', k)
                print(v)

        return direct_graph

    def find_association_greedy(self, lp_result_x):
        graph = self.build_graph(lp_result_x)

        result = []
        # just need to consider current node
        for end_node in range(self.layer_sizes[-1]):
            node = 'layer:{} node:{}'.format(self.len_layers - 1, end_node)
            path = ['end']
            while(node is not None):
                path.append(node)
                # [(node, weight), .....]
                next_nodes = graph[node]
                next_with_largest_weight = sorted(
                    next_nodes, key=lambda x: x[1], reverse=True)[0]
                node = next_with_largest_weight[0]

            path.reverse()
            result.append(path)

        return result


def relaxation_experiment():
    total_try = 10000
    
    success_count = 0
    solver_failure = 0

    for i in range(total_try):
        try:
            print('iter:', i)
            association = AssociationGraph()
            x, relaxation_success = association.lp()

            if relaxation_success:
                success_count += 1
        except:
            solver_failure += 1

    print('total try:', total_try, 
          'relaxation success count:', success_count, 
          'solver failures:', solver_failure)


def run_single():
    association = AssociationGraph()
    x, relaxation_success = association.lp()
    result = association.find_association_greedy(x)
    for p in result:
        print('path: ', p)


if __name__ == "__main__":
    # run_single()
    relaxation_experiment()
