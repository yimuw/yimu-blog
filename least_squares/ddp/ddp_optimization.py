import scipy.linalg as linalg
import numpy as np
from numpy.linalg import inv
import ddp_types

#Dynamic = ddp_types.LinearDynamic
 
Dynamic = ddp_types.NonlinearDynamic


class QuadraticCost:
    def __init__(self, mean, hessian):
        self.mean = mean
        self.hessian = hessian

    def eval(self, x):
        dx = x - self.mean
        return dx.T @ self.hessian @ dx

    def grad(self, x):
        dx = x -self.mean
        return 2 * self.hessian @ dx


class ControlLaw:
    def __init__(self, feedback, constant):
        self.feedback = feedback
        self.constant = constant


class DDP_optimization_perspective:
    def initialize(self):
        initial_state = np.array([0.1, 0.1])
        num_controls = 10
        init_controls = [np.array([0, 0.]) for i in range(num_controls)]
        target_state = np.array([2., 2.])
        return num_controls, initial_state, init_controls, target_state

    def forward_pass(self, num_controls, initial_state, init_controls):
        state = initial_state.copy()
        forward_pass_states = [state]
        for i in range(num_controls):
            next_state = Dynamic().f_function(
                state, init_controls[i])
            forward_pass_states.append(next_state)
            state = next_state
        return forward_pass_states

    def compute_ddp_subproblem_normal_equation(self, marginal_cost, xi_current, ui_current):
        SIZE_X = 2
        SIZE_U = 2
        system_size = SIZE_U + SIZE_X

        # varialbe order:
        #   [lhs ] xi = rhs
        #   [    ] ui
        rhs = np.zeros([system_size, system_size])
        lhs = np.zeros(system_size)
        # marginal cost: V(xj) = ||xj - mean||^2_w , V(xj = Ai * dxi + Bi * dui + f(xi, ui))
        residual_marginal = Dynamic().f_function(xi_current, ui_current) - marginal_cost.mean

        Ai = Dynamic().jacobi_wrt_state(xi_current, ui_current)
        Bi = Dynamic().jacobi_wrt_controls(xi_current, ui_current)
        jacobian_marginal_cost_wrt_to_xiui = np.zeros([SIZE_X, system_size])
        jacobian_marginal_cost_wrt_to_xiui[:, 0:2] = Ai
        jacobian_marginal_cost_wrt_to_xiui[:, 2:4] = Bi
        weight_marginal_cost = marginal_cost.hessian

        rhs += jacobian_marginal_cost_wrt_to_xiui.T @ weight_marginal_cost @ jacobian_marginal_cost_wrt_to_xiui
        lhs += - jacobian_marginal_cost_wrt_to_xiui.T @ weight_marginal_cost @ residual_marginal

        # ||ui + dui||^2
        weight_u = 0.5 * 1e-6
        rhs[2:4, 2:4] += 2 * weight_u * np.identity(2)
        lhs[2:4] += -2 * weight_u * ui_current
        return rhs, lhs

    def solve_ddp_subproblem(self, marginal_cost, xi_current, ui_current):
        rhs, lhs = self.compute_ddp_subproblem_normal_equation(marginal_cost, xi_current, ui_current)

        # |A1 A2| xi = b1
        # |A3 A4| ui   b2
        # note: A2 = A3.T
         
        # 1. elminate ui.
        #  (A1 - A2 * inv(A4) * A3) xi = b1 - A2 * inv(A4) * b2
        # 2. Gievn xi, ui is
        #  A3*xi + A4*ui = b2
        #  ui = inv(A4)*(b2 - A3*xi) = inv(A4)*b2 - inv(A4)*A3 * xi
        A1 = rhs[0:2, 0:2]
        A2 = rhs[0:2, 2:4]
        A3 = rhs[2:4, 0:2]
        A4 = rhs[2:4, 2:4]
        b1 = lhs[0:2]
        b2 = lhs[2:4]

        A4_inv = np.linalg.inv(A4)

        rhs_xi = A1 - A2 @ A4_inv @ A3
        lhs_xi = b1 - A2 @ A4_inv @ b2
        xi_star = np.linalg.solve(rhs_xi, lhs_xi)

        # the nonlinear derivation is very very trick! check notes.
        xi_marginal_cost = QuadraticCost(mean=xi_star + xi_current, hessian=rhs_xi)

        # print('mean:', xi_marginal_cost.mean)
        # print('w:', xi_marginal_cost.hessian)

        ui_control_law = ControlLaw(constant = A4_inv @ b2, feedback= - A4_inv @ A3)
        return xi_marginal_cost, ui_control_law


    def backward_pass(self, num_controls, forward_pass_states, init_controls,
                      final_cost):
        marginal_cost = QuadraticCost(final_cost.quad_mean(), final_cost.quad_weight())

        feedback_laws = [None] * num_controls
        # iterate [n-1, 0] to compute the control law
        for i in range(num_controls - 1, -1, -1):
            state_i = forward_pass_states[i]
            control_i = init_controls[i]

            marginal_cost, feedback_law = self.solve_ddp_subproblem(marginal_cost, state_i, control_i)
            feedback_laws[i] = feedback_law
        return feedback_laws

    def apply_control_law(self, num_controls, init_controls, forward_pass_states, feedback_laws):
        new_cur_state = forward_pass_states[0].copy()
        new_states = [new_cur_state]
        new_controls = []
        for i in range(num_controls):
            feedback_law = feedback_laws[i]
            dx = new_cur_state - forward_pass_states[i]
            # the argmin_u Q(u, x)
            du = feedback_law.constant + feedback_law.feedback @ dx

            step = 0.5
            control = init_controls[i] + step * du
            new_cur_state = Dynamic().f_function(new_cur_state, control)

            new_controls.append(control)
            new_states.append(new_cur_state)
        return new_controls, new_states

    def check_dynamic(self, num_controls, states, controls):
        state0 = states[0]

        integrated_states = self.forward_pass(num_controls, state0, controls)
        diff = np.stack(integrated_states) - np.stack(states)
        assert np.allclose(np.sum(diff), 0)
        # print('integrated_states - ddp_states: ', diff)

    def run(self):
        num_controls, initial_state, controls, target_state = self.initialize(
        )
        print('initial_state:', initial_state)
        print('target_state:', target_state)
        print('num_states:', num_controls + 1)

        for iter in range(10):

            forward_pass_states = self.forward_pass(num_controls, initial_state,
                                                    controls)

            # print('forward_pass_states:', forward_pass_states)

            final_state = forward_pass_states[-1]
            final_state_init_cost = ddp_types.TargetCost(final_state, target_state)
            feedback_laws = self.backward_pass(num_controls, forward_pass_states,
                                            controls, final_state_init_cost)

            controls, new_states = self.apply_control_law(
                num_controls, controls, forward_pass_states, feedback_laws)

            final_state_end_cost = ddp_types.TargetCost(
            new_states[-1], target_state)
            print('final_state_end_cost:', final_state_end_cost.cost())

        print('----------------------------------')
        print('new_controls:\n', controls)
        print('new_states:\n', new_states)

        self.check_dynamic(num_controls, new_states, controls)


def main():
    ddp = DDP_optimization_perspective()
    ddp.run()


if __name__ == "__main__":
    main()
