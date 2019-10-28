import scipy.linalg as linalg 
import numpy as np
import profiler
import fix_lag_types as t
import copy


# lag = 1. 
# Similar to Kalman filter without noise
class FixLagSmoother:
    def __init__(self, prior):
        self.state = prior.variables.copy()
        # Because I didn't specify weight, so weights are indentity
        self.state_hessian = np.identity(2)
        # Should be J * W * prior, but J = W = I(2)
        self.state_b = prior.variables.copy().reshape([2,1])

        self.all_states = [self.state]

    def get_all_states(self):
        return np.vstack(self.all_states)

    @profiler.time_it
    def optimize_for_new_measurement(self, distance_measurement):
        lhs, rhs = self.construct_linear_system(distance_measurement)
        # one step for linear system
        lhs_LU = linalg.lu_factor(lhs) 
        x = linalg.lu_solve(lhs_LU, -rhs)
        
        self.state = x[2:4].reshape([2])
        self.all_states.append(self.state)

        self.marginalization(lhs_LU, rhs)

        return x

    def construct_linear_system(self, distance_measurement):
        # x_i, x_i+1
        size_variables = 4
        # Hessian: A.T * W * A
        lhs = np.zeros([size_variables, size_variables])
        # A.T * W * r
        rhs = np.zeros([size_variables, 1])

        lhs[0:2, 0:2] += self.state_hessian
        rhs[0:2] += self.state_b
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
        

