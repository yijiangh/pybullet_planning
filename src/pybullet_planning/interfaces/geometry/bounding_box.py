import numpy as np
from collections import namedtuple
from itertools import product
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, UNKNOWN_FILE, OBJ_MESH_CACHE
from pybullet_planning.utils import implies


#####################################
# Bounding box

AABB = namedtuple('AABB', ['lower', 'upper'])
"""axis-aligned bounding box: https://en.wikipedia.org/wiki/Bounding_volume

Notice that the world-axis is used here. We don't have support for OOBB (using the object's local coordinate system)?

"""

def aabb_from_points(points):
    return AABB(np.min(points, axis=0), np.max(points, axis=0))

def aabb_union(aabbs):
    return aabb_from_points(np.vstack([aabb for aabb in aabbs]))

def aabb_overlap(aabb1, aabb2):
    lower1, upper1 = aabb1
    lower2, upper2 = aabb2
    return np.less_equal(lower1, upper2).all() and \
           np.less_equal(lower2, upper1).all()

#####################################
# Bounding box from body

def get_subtree_aabb(body, root_link=BASE_LINK):
    from pybullet_planning.interfaces.robots.link import get_link_subtree
    return aabb_union(get_aabb(body, link) for link in get_link_subtree(body, root_link))

def get_aabbs(body):
    from pybullet_planning.interfaces.robots.link import get_all_links
    return [get_aabb(body, link=link) for link in get_all_links(body)]

def get_aabb(body, link=None):
    # Note that the query is conservative and may return additional objects that don't have actual AABB overlap.
    # This happens because the acceleration structures have some heuristic that enlarges the AABBs a bit
    # (extra margin and extruded along the velocity vector).
    # Contact points with distance exceeding this threshold are not processed by the LCP solver.
    # AABBs are extended by this number. Defaults to 0.02 in Bullet 2.x
    #p.setPhysicsEngineParameter(contactBreakingThreshold=0.0, physicsClientId=CLIENT)
    if link is None:
        aabb = aabb_union(get_aabbs(body))
    else:
        aabb = p.getAABB(body, linkIndex=link, physicsClientId=CLIENT)
    return aabb

get_lower_upper = get_aabb

def get_aabb_center(aabb):
    lower, upper = aabb
    return (np.array(lower) + np.array(upper)) / 2.

def get_aabb_extent(aabb):
    """return the bounding box range in the x, y, z in the body's pose frame

    Parameters
    ----------
    aabb : AABB
        [description]

    Returns
    -------
    np array of three float
        [width, length, height]
    """
    lower, upper = aabb
    return np.array(upper) - np.array(lower)

def get_center_extent(body, **kwargs):
    aabb = get_aabb(body, **kwargs)
    return get_aabb_center(aabb), get_aabb_extent(aabb)

def aabb2d_from_aabb(aabb):
    (lower, upper) = aabb
    return lower[:2], upper[:2]

def aabb_contains_aabb(contained, container):
    lower1, upper1 = contained
    lower2, upper2 = container
    return np.less_equal(lower2, lower1).all() and \
           np.less_equal(upper1, upper2).all()
    #return np.all(lower2 <= lower1) and np.all(upper1 <= upper2)

def aabb_contains_point(point, container):
    lower, upper = container
    return np.less_equal(lower, point).all() and \
           np.less_equal(point, upper).all()
    #return np.all(lower <= point) and np.all(point <= upper)

def get_bodies_in_region(aabb):
    """This query will return all the unique ids of objects that have axis aligned bounding box overlap with a given axis aligned bounding box.

    Note that the query is conservative and may return additional objects that don't have actual AABB overlap.
    This happens because the acceleration structures have some heuristic that enlarges the AABBs a bit
    (extra margin and extruded along the velocity vector).

    Parameters
    ----------
    aabb : [type]
        [description]

    Returns
    -------
    a list of object unique ids.
    """
    (lower, upper) = aabb
    return p.getOverlappingObjects(lower, upper, physicsClientId=CLIENT)

def get_aabb_volume(aabb):
    return np.prod(get_aabb_extent(aabb))

def get_aabb_area(aabb):
    return np.prod(get_aabb_extent(aabb2d_from_aabb(aabb)))

#####################################

# AABB approximation
def get_aabb_vertices(aabb):
    d = len(aabb[0])
    return [tuple(aabb[i[k]][k] for k in range(d))
            for i in product(range(len(aabb)), repeat=d)]
