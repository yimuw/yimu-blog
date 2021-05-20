import time
import numpy as np
from collections import defaultdict

time_map = defaultdict(list)


def time_it(func):
    def inner(*args, **kwargs):
        time_start = time.time()
        res = func(*args, **kwargs)
        elapsed_time = time.time() - time_start

        key = func.__name__
        time_map[key].append(elapsed_time)
        return res

    return inner


def print_time_map():
    for k, v in time_map.items():
        print('{}: mean time: {},  # called: {}, max: {}'.format(
            k, np.mean(v), len(v), max(v)))
