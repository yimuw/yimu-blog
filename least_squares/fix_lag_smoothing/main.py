import scipy.linalg as linalg
import numpy as np
import fix_lag_types as t
import batch_optimizer
import fix_lag_smoother
import profiler


def simulate_data(num_states):
    # movement = x2 - x1
    movement = np.array([2., 1.])

    states_gt = [t.State(i * movement) for i in range(num_states)]
    distrance_measurements = [t.DistanceBetweenStates(state1_index=i, state2_index=i + 1,
                                                      distance=movement + 5 * np.random.rand()) for i in range(num_states - 1)]

    state_guess = [t.State(movement) for i in range(num_states)]

    return states_gt, distrance_measurements, state_guess


def fix_lag_smoothing_demo():
    NUM_STATES = 20
    states_gt, distrance_measurements, state_guess = simulate_data(NUM_STATES)

    batch_optimization = batch_optimizer.BatchOptimization(
        state_guess, distrance_measurements)
    x = batch_optimization.optimize()
    batch_result = x.reshape(NUM_STATES, 2)
    print('batch :\n', x.reshape(NUM_STATES, 2))

    fix_lag = fix_lag_smoother.FixLagSmoother(states_gt[0])
    for distance in distrance_measurements:
        fix_lag.optimize_for_new_measurement(distance)
    fixed_lag_result = fix_lag.get_all_states()
    print('fixed lag :\n', fixed_lag_result)

    print('diff :\n', fixed_lag_result - batch_result)

    profiler.print_time_map()


if __name__ == "__main__":
    fix_lag_smoothing_demo()
