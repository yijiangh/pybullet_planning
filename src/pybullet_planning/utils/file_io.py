"""
file I/O
"""

import os
import pickle
import json
import datetime
from .shared_const import DATE_FORMAT

SEPARATOR = '\n' + 50*'-' + '\n'

#def inf_generator():
#    return iter(int, 1)
# inf_generator = count

def print_separator(n=50):
    print('\n' + n*'-' + '\n')

def read(filename):
    with open(filename, 'r') as f:
        return f.read()

def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)

def read_pickle(filename):
    """[summary]

    Parameters
    ----------
    filename : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """
    # Can sometimes read pickle3 from python2 by calling twice
    # Can possibly read pickle2 from python3 by using encoding='latin1'
    with open(filename, 'rb') as f:
        return pickle.load(f)

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    """[summary]
    """
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

def get_date():
    return datetime.datetime.now().strftime(DATE_FORMAT)

def load_yaml(path):
    import yaml
    # grep -r --include="*.py" "yaml\." *
    # yaml.dump()
    with open(path, 'r') as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise exc
