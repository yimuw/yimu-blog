import scipy.linalg as linalg 
import numpy as np
import fix_lag_types as t
import batch_opt
import profiler


def simulate_data():
    # movement = x2 - x1
    movement = np.array([2., 1.])
    NUM_STATES = 100
    
    states_gt = [t.State(i * movement) for i in range(NUM_STATES)]
    distrance_gt = [movement for i in range(NUM_STATES - 1)]

    state_guess = [t.State(movement) for i in range(NUM_STATES)]

    return states_gt, distrance_gt, state_guess

if __name__ == "__main__":
    states_gt, distrance_gt, state_guess = simulate_data()
    batch_optimization = batch_opt.BatchOptimization(state_guess, distrance_gt)
    x = batch_optimization.optimize()
    print('x:', x.T)

    profiler.print_time_map()