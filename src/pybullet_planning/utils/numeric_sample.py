import numpy as np
import random
from itertools import cycle, islice

from .shared_const import INF

def clip(value, min_value=-INF, max_value=+INF):
    return min(max(min_value, value), max_value)

def randomize(sequence): # TODO: bisect
    indices = list(range(len(sequence)))
    random.shuffle(indices)
    for i in indices:
        try:
            yield sequence[i]
        except TypeError:
            yield list(sequence)[i]

def get_random_seed():
    return random.getstate()[1][0]

def get_numpy_seed():
    return np.random.get_state()[1][0]

def set_random_seed(seed):
    if seed is not None:
        random.seed(seed)

def set_numpy_seed(seed):
    # These generators are different and independent
    if seed is not None:
        np.random.seed(seed % (2**32))
        #print('Seed:', seed)

def roundrobin(*iterables):
    # https://docs.python.org/3.1/library/itertools.html#recipes
    "roundrobin('ABC', 'D', 'EF') --> A D E B F C"
    # Recipe credited to George Sakkis
    pending = len(iterables)
    nexts = cycle(iter(it).__next__ for it in iterables)
    while pending:
        try:
            for next in nexts:
                yield next()
        except StopIteration:
            pending -= 1
            nexts = cycle(islice(nexts, pending))

def chunks(sequence, n=1):
    for i in range(0, len(sequence), n):
        yield sequence[i:i + n]
