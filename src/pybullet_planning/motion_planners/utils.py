import numpy as np
from random import shuffle
from itertools import islice
from collections import deque
import time
from pybullet_planning.interfaces.env_manager.pose_transformation import get_difference, get_unit_vector

__all__ = [
    'compute_path_cost',
]

INF = float('inf')

# default RRT configurations
RRT_ITERATIONS = 20
RRT_RESTARTS = 2
RRT_SMOOTHING = 20


def irange(start, stop=None, step=1):  # np.arange
    if stop is None:
        stop = start
        start = 0
    while start < stop:
        yield start
        start += step


def negate(test):
    return lambda *args, **kwargs: not test(*args, **kwargs)


def argmin(function, sequence):
    # TODO: use min
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]


def get_pairs(lst):
    return zip(lst[:-1], lst[1:])


def merge_dicts(*args):
    result = {}
    for d in args:
        result.update(d)
    return result
    # return dict(reduce(operator.add, [d.items() for d in args]))


def flatten(iterable_of_iterables):
    return (item for iterables in iterable_of_iterables for item in iterables)


def randomize(sequence):
    shuffle(sequence)
    return sequence


def bisect(sequence):
    sequence = list(sequence)
    indices = set()
    queue = deque([(0, len(sequence)-1)])
    while queue:
        lower, higher = queue.popleft()
        if lower > higher:
            continue
        index = int((lower + higher) / 2.)
        assert index not in indices
        #if is_even(higher - lower):
        yield sequence[index]
        queue.extend([
            (lower, index-1),
            (index+1, higher),
        ])


def take(iterable, n=INF):
    if n == INF:
        n = None  # NOTE - islice takes None instead of INF
    elif n == None:
        n = 0  # NOTE - for some of the uses
    return islice(iterable, n)


def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    enums['names'] = sorted(enums.keys(), key=lambda k: enums[k])
    return type('Enum', (), enums)

def elapsed_time(start_time):
    return time.time() - start_time

######################################

def inf_sequence():
    return iter(int, 1)


def compute_path_cost(path, cost_fn):
    """compute the total cost of a given path using the given cost function

    Parameters
    ----------
    path :
        input path
    cost_fn : function handle
        cost function - ``cost_fn(q)->float``

    Returns
    -------
    float
        total cost
    """
    if path is None:
        return INF
    return sum(cost_fn(*pair) for pair in get_pairs(path))


def remove_redundant(path, tolerance=1e-3):
    """Remove redundant waypoints in a given path if consecutive points are too close under L2 norm.
    """
    assert path
    new_path = [path[0]]
    for conf in path[1:]:
        difference = np.array(new_path[-1]) - np.array(conf)
        if not np.allclose(np.zeros(len(difference)), difference, atol=tolerance, rtol=0):
            new_path.append(conf)
    return new_path


def waypoints_from_path(path, difference_fn=None, tolerance=1e-3):
    """Remove redundant waypoints in a given path if consecutive points are too close under L2 norm.
    """
    difference_fn = difference_fn or get_difference
    path = remove_redundant(path, tolerance=tolerance)
    if len(path) < 2:
        return path
    waypoints = [path[0]]
    last_conf = path[1]
    last_difference = get_unit_vector(difference_fn(last_conf, waypoints[-1]))
    for conf in path[2:]:
        difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        if not np.allclose(last_difference, difference, atol=tolerance, rtol=0):
            waypoints.append(last_conf)
            difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        last_conf = conf
        last_difference = difference
    waypoints.append(last_conf)
    return waypoints


def convex_combination(x, y, w=0.5):
    return (1-w)*np.array(x) + w*np.array(y)

##################################################

def forward_selector(path):
    return path


def backward_selector(path):
    return reversed(list(path))


def random_selector(path):
    return randomize(path)


def bisect_selector(path):
    return bisect(path)


default_selector = bisect_selector
