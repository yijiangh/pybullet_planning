import numpy as np

# constants
INF = np.inf
PI = np.pi
CIRCULAR_LIMITS = -PI, PI
UNBOUNDED_LIMITS = -INF, INF
DEFAULT_TIME_STEP = 1./240. # seconds

# pybullet setup parameters
CLIENTS = {}
CLIENT = 0

INFO_FROM_BODY = {}

# colors
RED = (1, 0, 0, 1)
GREEN = (0, 1, 0, 1)
BLUE = (0, 0, 1, 1)
BLACK = (0, 0, 0, 1)
WHITE = (1, 1, 1, 1)
BROWN = (0.396, 0.263, 0.129, 1)
TAN = (0.824, 0.706, 0.549, 1)
GREY = (0.5, 0.5, 0.5, 1)
YELLOW = (1, 1, 0, 1)

GRAVITY = 9.8

# links
BASE_LINK = -1
STATIC_MASS = 0

# geometry
DEFAULT_MESH = ''
DEFAULT_EXTENTS = [1, 1, 1]
DEFAULT_RADIUS = 0.5
DEFAULT_HEIGHT = 1
DEFAULT_SCALE = [1, 1, 1]
DEFAULT_NORMAL = [0, 0, 1]

# planning
DEFAULT_RESOLUTION = 0.05

# Collision
# MAX_DISTANCE = 1e-3
# Used in collision checking query, e.g. pybullet.getClosestPoint
# If the distance between objects exceeds this maximum distance, no points may be returned.
MAX_DISTANCE = 0.

UNKNOWN_FILE = 'unknown_file'
NULL_ID = -1
TEMP_DIR = 'temp/'

OBJ_MESH_CACHE = {}
"""a cache keeping track of loaded obj objects

"""

def get_client(client=None):
    if client is None:
        return CLIENT
    return client

def set_client(client):
    global CLIENT
    CLIENT = client
