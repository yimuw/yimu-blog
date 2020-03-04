import scipy.linalg as linalg
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import ddp_types


# cost (u1, u2, u3) = k||u1||^2 + k||u2||^2 + k||u3||^3 + ||x4 - target||^2
#   subject to: x_i+1 = A_i * x_i + B_i * u_i
#               x_0 = start
class direct_DDP:
    def initialize(self):
        initial_state = np.array([0.5, 2.])
        num_controls = 3
        init_controls = [np.array([5, 5.]) for i in range(num_controls)]
        target_state = np.array([10, 10.])
        return num_controls, initial_state, init_controls, target_state
    
    def forward_pass(self, num_controls, initial_state, controls):
        state = initial_state.copy()
        forward_pass_states = [state]
        for i in range(num_controls):
            next_state = ddp_types.Dynamic().f_function(
                state, controls[i])
            forward_pass_states.append(next_state)
            state = next_state
        return forward_pass_states

    def almost_ddp(self, num_controls, initial_state, init_controls, target_state):
        jacobian, residual = self.using_equality_constraints_for_target_residual(num_controls, initial_state, init_controls, target_state)
        print(jacobian, residual)

        lhs = jacobian.T @ jacobian
        rhs = - jacobian.T @ residual

        lhs, rhs = self.add_controls_cost(lhs, rhs, num_controls, init_controls)

        controls_result = np.linalg.solve(lhs, rhs)
        controls_result = controls_result.reshape(num_controls, 2)
        print('control_result,', controls_result)

        return controls_result

    def add_controls_cost(self, lhs, rhs, num_controls, init_controls):
        # cost for 0.5 * k||u_i||^2
        K = 1e-6

        for i in range(num_controls):
            var_idx = i * 2
            lhs[var_idx: var_idx + 2, var_idx: var_idx + 2] += 2 * K * np.identity(2)
            rhs[var_idx: var_idx + 2, :] += - 2 * K * init_controls[i].reshape([2,1])
        
        return lhs, rhs

    
    def using_equality_constraints_for_target_residual(self, num_controls, initial_state, init_controls, target_state):
        SIZE_OF_CONTROL = 2

        jacobian = np.zeros([2, num_controls * SIZE_OF_CONTROL])

        states = self.forward_pass(num_controls, initial_state, init_controls)
        print("predicted states:", states)

        mul_A = np.identity(SIZE_OF_CONTROL)

        # B_{n-1} * u_{n-1}
        state_idx = num_controls - 1
        var_idx = SIZE_OF_CONTROL * state_idx
        B_last = ddp_types.Dynamic().jacobi_wrt_controls(states[-2], init_controls[-1])
        jacobian[:, var_idx:] += B_last

        # (prob_{j=i+1}^{n-1} A_j) * B_i * u_i
        for state_idx in range(num_controls-2, -1, -1):
            print(state_idx)
            var_idx = SIZE_OF_CONTROL * state_idx
            A_i = ddp_types.Dynamic().jacobi_wrt_state(states[state_idx], init_controls[state_idx])
            mul_A = mul_A @ A_i
            B_i = ddp_types.Dynamic().jacobi_wrt_controls(states[state_idx], init_controls[state_idx])
            jacobian[:, var_idx:var_idx+SIZE_OF_CONTROL] += mul_A @ B_i

        # check DDP note
        # r = g(u1, u2, u3) - t
        # r = g(0,0,0) - t + dgdu * u
        # g(0,0,0) = (\prod A_i) * x_0
        last_state = states[-1]
        A_0 = ddp_types.Dynamic().jacobi_wrt_state(states[0], init_controls[0])
        residual = (mul_A @ A_0 @ initial_state - target_state).reshape([2,1])
        
        return jacobian, residual

    def check_dynamic(self, num_controls, states, controls):
        state0 = states[0]

        integrated_states = self.forward_pass(num_controls, state0, controls)
        diff = np.stack(integrated_states) - np.stack(states)
        assert np.allclose(np.sum(diff), 0)
        print('integrated_states - ddp_states: ', diff)
        

    def run(self):
        num_controls, initial_state, init_controls, target_state = self.initialize()
        
        controls_result = self.almost_ddp(num_controls, initial_state, init_controls, target_state)

        result_states = self.forward_pass(num_controls, initial_state, controls_result)

        final_state_end_cost = ddp_types.TargetCost(result_states[-1], target_state)
        print('final_state_end_cost:', final_state_end_cost.cost())

        self.check_dynamic(num_controls, result_states, controls_result)


def main():
    ddp = direct_DDP()
    ddp.run()


if __name__ == "__main__":
    main()