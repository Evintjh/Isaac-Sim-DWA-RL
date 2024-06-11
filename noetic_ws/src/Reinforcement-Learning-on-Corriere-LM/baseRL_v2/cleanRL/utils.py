import random
import torch
import numpy as np
from functools import wraps
import time
from datetime import timedelta
from matplotlib import pyplot as plt


def set_random_seed(seed) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time

        if kwargs:
            print(f'Function {func.__name__}{args} {kwargs} ----> {str(timedelta(seconds=total_time))} seconds \n\n\n')
        else:
            print(f'Function {func.__name__}{args} -----> {str(timedelta(seconds=total_time))} seconds \n\n\n')
        return result

    return timeit_wrapper


def plot_results(results, output_path, envs_name, eval_frequency):
    plt.figure()
    temp_results = {}
    for timestep, timestep_results_seed in enumerate(results):
        if timestep in temp_results:
            temp_results[timestep].extend(timestep_results_seed)
        else:
            temp_results[timestep] = timestep_results_seed
    x, z = zip(*sorted(temp_results.items()))
    x = [xi * eval_frequency for xi in x]
    y = [np.mean(i) for i in z]
    y_std = [np.std(i) for i in z]
    plt.plot(x, y)  # next(cycol)
    plt.fill_between(x, np.subtract(y, y_std), np.add(y, y_std), alpha=.2)
    plt.legend()
    plt.savefig('%s/%s.png' % (output_path, envs_name))
