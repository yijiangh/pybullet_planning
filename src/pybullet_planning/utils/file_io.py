"""
file I/O
"""

import os, platform
import pickle
import json
import random
import datetime
import numpy as np

from pybullet_planning.utilities import INF

SEPARATOR = '\n' + 50*'-' + '\n'

#def inf_generator():
#    return iter(int, 1)
# inf_generator = count

def print_separator(n=50):
    print('\n' + n*'-' + '\n')

def is_remote():
    return 'SSH_CONNECTION' in os.environ

def is_darwin(): # TODO: change loading accordingly
    return platform.system() == 'Darwin' # platform.release()
    #return sys.platform == 'darwin'

def read(filename):
    with open(filename, 'r') as f:
        return f.read()

def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)

def read_pickle(filename):
    # Can sometimes read pickle3 from python2 by calling twice
    # Can possibly read pickle2 from python3 by using encoding='latin1'
    with open(filename, 'rb') as f:
        return pickle.load(f)

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def read_json(path):
    return json.loads(read(path))

def write_json(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=2, sort_keys=True)

def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)

def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)

def safe_zip(sequence1, sequence2):
    assert len(sequence1) == len(sequence2)
    return zip(sequence1, sequence2)

def clip(value, min_value=-INF, max_value=+INF):
    return min(max(min_value, value), max_value)

def randomize(sequence): # TODO: bisect
    indices = range(len(sequence))
    random.shuffle(indices)
    for i in indices:
        yield sequence[i]

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

def get_date():
    return datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')

def implies(p1, p2):
    return not p1 or p2
