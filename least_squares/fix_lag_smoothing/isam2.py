import scipy.linalg as linalg 
import numpy as np
import profiler
import fix_lag_types as t
import copy


class EliminationSchur:
    def __init__(self, A_inv, B, b1, start_state_idx, end_state_idx):
        """
         [A, B;   x1   b1
          C, D] * x2 = b2

          start_state_idx := idx of x1
          start_state_idx := idx of x2
          Note: x2 is always a single state, whereas x1 can be one or more states.
        """
        self.A_inv = A_inv
        self.B = B
        self.b1 = b1
        
    def compute_x1(self, x2):
        assert(x2.size() == 2)
        x1 = self.A_inv @ (self.b1 - self.B @ x2)
        return x1

class EliminationTreeNode:
    def __init__(self, next, schur_block):
        self.next = next
        self.schur_block = schur_block

class EliminationTree:
    """
        Save the elimination order for each state
        It is actually a list for each state. A tree is a more efficent representation for all states.
    """
    def __init__(self, prior_block):
        # dummy node
        self.tree_lookup = {0 : EliminationTreeNode(None, None)}

    def add_elimination_order(self, schur_block):
        """
            eliminaion_order(xn) = [schur_block for state xn to xj] + eliminaion_order[xj]
        """
        start_state_idx = schur_block.start_state_idx
        end_state_idx = schur_block.end_state_idx
        assert(end_state_idx not in self.tree_lookup)
        self.tree_lookup[end_state_idx] = EliminationTreeNode(self.tree_lookup[start_state_idx], schur_block)

    def back_substitude_given_x_end(self, x_end, x_end_index):
        tree_node = self.tree_lookup[x_end_index]

        x2 = x_end
        x_result = []
        while(tree_node.next is not None):
            x1 = tree_node.schur_block.compute_x1(x2)
            x_result.append(x1)
            x2 = x1[:2]
        
        x_result = np.vstack(x_result.reverse())
        assert(x_result.size() == (x_end_index - 1) * 2)
        return x_result


class MarginalizationLookup:
    """
        For marginal prior lookup
    """
    def __init__(self):
        self.marginal_states = {}

    def add_marginal_state(self, state_idx, marginal_state):
        assert(state_idx not in self.marginal_states)
        self.marginal_states[state_idx] = marginal_state
    
    def get_state_at(self, index):
        assert(index not in self.marginal_states)
        return self.marginal_states[index]


class ISAM2:
    def __init__(self, prior):
        self.state = prior.variables.copy()
        # Because I didn't specify weight, so weights are indentity
        self.state_hessian = np.identity(2)
        # Should be J * W * prior, but J = W = I(2)
        self.state_b = prior.variables.copy().reshape([2,1])

        self.all_states = [self.state]
        self.all_measurements = []

    def get_all_states(self):
        return np.vstack(self.all_states)

    @profiler.time_it
    def optimize_for_distance_measurement(self, distance_measurement):
        if self.is_loop_closure(distance_measurement):
            lhs, rhs = self.construct_linear_system_for_loop_closure(distance_measurement)
        # odometry measurement
        else:
            lhs, rhs = self.construct_linear_system_for_odometry(distance_measurement)
        # one step for linear system
        self.solve_and_update(lhs, rhs)

    def is_loop_closure(self, distance_measurement):
        return distance_measurement.state1_index + 1 != distance_measurement.state2_index

    def construct_linear_system_for_loop_closure(self, distance_measurement):
        return None, None

    def construct_linear_system_for_odometry(self, distance_measurement):
        # x_i, x_i+1
        size_variables = 4
        # Hessian: A.T * W * A
        lhs = np.zeros([size_variables, size_variables])
        # A.T * W * r
        rhs = np.zeros([size_variables, 1])

        lhs[0:2, 0:2] += self.state_hessian
        rhs[0:2] += self.state_b
        # initial states doesn't matter for linear system
        dm = t.DistanceMeasurement(t.State(self.state.copy()), t.State(self.state.copy()), distance_measurement)
        jacobi_wrt_s1 = dm.jacobi_wrt_state1()
        jacobi_wrt_s2 = dm.jacobi_wrt_state2()
        jacobi_dm = np.hstack([jacobi_wrt_s1, jacobi_wrt_s2])
        lhs[0:4, 0:4] += jacobi_dm.T @ jacobi_dm
        rhs[0:4] += jacobi_dm.T @ dm.residual()

        return lhs, rhs

    def marginalization(self, lhs_LU, rhs):
        """
            factor-graph-for-robot-perception, page 68
        """
        hessian_LU = lhs_LU
        # By definition, covariance = inv(hessian)
        # A * A-1 = I, reuse LU
        covariance = linalg.lu_solve(hessian_LU, np.identity(4))
        mean_covariance_form = covariance @ rhs

        self.state_hessian = np.linalg.inv(covariance[2:4, 2:4])
        # This is tricky
        # get it by equating ||Ax + b||^2 = ||x + mu ||^2_W
        self.state_b = self.state_hessian @ mean_covariance_form[2:4]

        x_test = np.linalg.solve(self.state_hessian, -self.state_b)
        np.testing.assert_array_almost_equal(x_test.reshape([2]), self.state)
        

