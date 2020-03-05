import scipy.linalg as linalg
import numpy as np
from numpy.linalg import inv
import ddp_types

Dynamic = ddp_types.LinearDynamic

class DPPstate:
    def __init__(self, v_quadratic_weight, v_quadratic_mean):
        self.v_quadratic_mean = v_quadratic_mean.copy()
        self.v_quadratic_weight = v_quadratic_weight.copy()

    def compute_q(self, state, control):
        dynamic = Dynamic()
        # predict_x = dynamic.f_function(state, control)
        # print('state, control:', state, control)
        # print('predict_x:', predict_x)

        dynamic_jacobi_wrt_state = dynamic.jacobi_wrt_state(state, control)
        dynamic_jacobi_wrt_control = dynamic.jacobi_wrt_controls(
            state, control)
        dynamic_jacobi = np.hstack(
            [dynamic_jacobi_wrt_state, dynamic_jacobi_wrt_control])

        # Q_n(x,u) = l_n(x, u) + V_n+1(f(x,u))
        q_hessian_v_term = dynamic_jacobi.T @ self.v_quadratic_weight @ dynamic_jacobi
        q_grad_v_term = dynamic_jacobi.T @ self.v_quadratic_weight @ (
            -self.v_quadratic_mean)

        # assume l(x, u) = 0.5 * k * u.T @ u
        k = 1e-6
        q_hessian_l_term = np.diag([0, 0, k, k])
        ux, uy = control
        q_grad_l_term = np.array([0, 0, k * ux, k * uy])

        # Q_n(x,u) = l_n(x, u) + V_n+1(f(x,u))
        self.q_hessian = q_hessian_v_term + q_hessian_l_term
        self.q_grad = -(q_grad_v_term + q_grad_l_term)

    def compute(self, state, control):
        self.compute_q(state, control)

        # The q system:
        # | A11 A12 | x = b1
        # | A21 A22 | u   b2
        # 1. u given x:
        #    u = inv(A22)* (b2 - A21 * x)
        #    u = inv(A22) * b2 - inv(A22) * A21 * x
        #               |                |
        #              u_k1             u_k2
        #
        # 2. eliminate u
        #    u = inv(A22) * b2 - inv(A22) * A21 * x
        #
        #    A11 * x + A12 * u = b1
        #    plug in u,
        #    A11 * x + A12 * (inv(A22) * b2 - inv(A22) * A21 * x) = b1
        #    (A11 - A12 @ inv(A22) @ A21) x = b1 - A12 @ inv(A22) @ b2
        #

        A11 = self.q_hessian[0:2, 0:2]
        A12 = self.q_hessian[0:2, 2:4]
        A21 = self.q_hessian[2:4, 0:2]
        A22 = self.q_hessian[2:4, 2:4]
        b1 = self.q_grad[0:2]
        b2 = self.q_grad[2:4]

        # compute u given x.
        # u = u_k1 + u_k2 * x
        A22_inv = inv(A22)
        self.u_k1 = A22_inv @ b2
        self.u_k2 = -A22_inv @ A21

        # eliminate u for q function, the result is the next v function.
        self.v_next_quad_weight = A11 - A12 @ A22_inv @ A21
        self.v_next_quad_b = b1 - A12 @ A22_inv @ b2
        self.v_next_quad_mean = np.linalg.solve(self.v_next_quad_weight,
                                                self.v_next_quad_b)
        return self.v_next_quad_weight, self.v_next_quad_mean


class DDP:
    def initialize(self):
        initial_state = np.array([0.5, 2.])
        num_controls = 3
        init_controls = [np.array([5, 5.]) for i in range(num_controls)]
        target_state = np.array([10, 10.])
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

    def backward_pass(self, num_controls, forward_pass_states, init_controls,
                      final_cost):
        v_quad_weight = final_cost.quad_weight()
        v_quad_mean = final_cost.quad_mean()

        ddp_states = [None] * num_controls
        # iterate [n-1, 0] to compute the control law
        for i in range(num_controls - 1, -1, -1):
            state = forward_pass_states[i]
            control = init_controls[i]
            ddp_state = DPPstate(v_quad_weight, v_quad_mean)
            v_quad_weight, v_quad_mean = ddp_state.compute(state, control)
            ddp_states[i] = ddp_state
        return ddp_states

    def apply_control_law(self, num_controls, initial_state, ddp_states):
        state = initial_state.copy()
        new_states = [state]
        new_controls = []
        for i in range(num_controls):
            ddp_state = ddp_states[i]
            # the argmin_u Q(u, x)
            control = ddp_state.u_k1 + ddp_state.u_k2 @ state
            state = Dynamic().f_function(state, control)

            new_controls.append(control)
            new_states.append(state)
        return new_controls, new_states

    def check_dynamic(self, num_controls, states, controls):
        state0 = states[0]

        integrated_states = self.forward_pass(num_controls, state0, controls)
        diff = np.stack(integrated_states) - np.stack(states)
        assert np.allclose(np.sum(diff), 0)
        print('integrated_states - ddp_states: ', diff)

    def run(self):
        num_controls, initial_state, init_controls, target_state = self.initialize(
        )
        print('initial_state:', initial_state)
        print('target_state:', target_state)
        print('num_states:', num_controls + 1)

        forward_pass_states = self.forward_pass(num_controls, initial_state,
                                                init_controls)

        print('forward_pass_states:', forward_pass_states)

        final_state = forward_pass_states[-1]
        final_state_init_cost = ddp_types.TargetCost(final_state, target_state)
        print('final_state_init_cost:', final_state_init_cost.cost())

        ddp_states = self.backward_pass(num_controls, forward_pass_states,
                                        init_controls, final_state_init_cost)

        new_controls, new_states = self.apply_control_law(
            num_controls, initial_state, ddp_states)
        print('----------------------------------')
        print('new_controls:', new_controls)
        print('new_states:', new_states)

        final_state_end_cost = ddp_types.TargetCost(new_states[-1], target_state)
        print('final_state_end_cost:', final_state_end_cost.cost())

        self.check_dynamic(num_controls, new_states, new_controls)


def main():
    ddp = DDP()
    ddp.run()


if __name__ == "__main__":
    main()