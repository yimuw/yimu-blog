import numpy as np
import matplotlib.pyplot as plt

import extented_kalman_filter
import kalman_least_sqr
import batch_least_sqr
import model

def plot_all():
    init_state = np.array([4.0, 0.1, 1, 0, 0.5])
    gt_states, gt_measurements = model.generate_gt_data(init_state)

    print('=========run_kalman_filter==========')
    extented_kalman_filter.run_extented_kalman_filter(gt_states, gt_measurements)

    print('=========run_kalman_least_sqr, iter : 1 ==========')
    kalman_least_sqr.run_kalman_least_sqr(gt_states, gt_measurements, max_iter = 1)

    print('=========run_kalman_least_sqr, iter : 10 ==========')
    kalman_least_sqr.run_kalman_least_sqr(gt_states, gt_measurements, max_iter = 10)

    print('==========run_graph_filter===========')
    batch_least_sqr.run_batch_least_sqr(gt_states, gt_measurements)
    plt.show()


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    np.random.seed(0)
    plot_all()