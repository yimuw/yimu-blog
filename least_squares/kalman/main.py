import numpy as np
import matplotlib.pyplot as plt

import extented_kalman_filter
import kalman_least_sqr
import batch_least_sqr

def plot_all():
    print('=========run_kalman_filter==========')
    init_state = np.array([0.0, 0.0, 1, 0, 0.5])
    extented_kalman_filter.run_extented_kalman_filter(init_state)

    print('=========run_kalman_plus_filter==========')
    kalman_least_sqr.run_kalman_plus_filter(init_state)

    print('==========run_graph_filter===========')
    batch_least_sqr.run_batch_least_sqr(init_state)
    plt.show()


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    plot_all()