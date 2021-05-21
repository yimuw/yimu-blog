import scipy.linalg as linalg
import numpy as np
import fix_lag_types
import batch_optimizer
import fix_lag_smoother
import profiler


def simulate_data(num_states):
    # movement = x2 - x1
    state_init = fix_lag_types.State(np.array([0., 0., 1., 2.]))

    states_gt = [state_init]
    for i in range(num_states - 1):
        states_gt.append(states_gt[-1].predict())
    states_gt = [s.variables for s in states_gt]

    odometry_measurements = [fix_lag_types.OdometryMeasurement(
        state1_index=i, state2_index=i + 1) for i in range(num_states - 1)]

    gps_measurements = [fix_lag_types.GPSMeasurement(state_index=i,
                                                     gps=states_gt[i][:2] + 0.5 * np.random.rand(2))
                        for i in range(num_states)]

    prior_measurement = np.array([0., 0., 1., 2.])

    state_init_guess = [fix_lag_types.State(
        np.random.random(4)) for i in range(num_states)]

    return states_gt, state_init_guess, odometry_measurements, gps_measurements, prior_measurement


def fix_lag_smoothing_demo():
    NUM_STATES = 30
    states_gt, state_init_guess, odometry_measurements, gps_measurements, prior_measurement = simulate_data(
        NUM_STATES)

    if False:
        print('states_gt :\n', states_gt)
        print('odometry_measurements:', odometry_measurements)
        print('gps_measurements:', gps_measurements)
        print('state_guess:', state_init_guess)

    batch_optimization = batch_optimizer.BatchOptimization(
        state_init_guess, odometry_measurements, gps_measurements, prior_measurement)
    batch_result, batch_cov = batch_optimization.optimize()

    # format data
    batch_result = np.vstack([b.variables for b in batch_result])
    batch_cov_diag = np.diag(batch_cov).reshape(-1, 4)
    states_gt = np.vstack(states_gt)

    print('states_gt :\n', states_gt)
    print('batch resutl:\n', batch_result)
    print('batch cov diag:\n', batch_cov_diag)
    fix_lag = fix_lag_smoother.FixLagSmoother(
        prior_measurement, gps_measurements[0])

    # observation 1 by 1. e.g. real-time simulation
    for i in range(len(odometry_measurements)):
        fix_lag.optimize_for_new_measurement(
            odometry_measurements[i], gps_measurements[i+1])

    fixed_lag_result = fix_lag.get_all_states()
    fixed_lag_cov_diag = fix_lag.get_diag_cov()
    print('fixed lag :\n', fixed_lag_result)
    print('fixed lag cov\n:', fixed_lag_cov_diag)

    # show diff.
    print('cov diff:\n', fixed_lag_cov_diag - batch_cov_diag)
    print('fixed lag - batch\n:', fixed_lag_result - batch_result)

    profiler.print_time_map()


if __name__ == "__main__":
    fix_lag_smoothing_demo()
