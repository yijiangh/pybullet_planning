import numpy as np
from collections import namedtuple
from itertools import product
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, UNKNOWN_FILE, OBJ_MESH_CACHE
from pybullet_planning.utils import implies
from pybullet_planning.interfaces.robots.link import get_link_subtree, get_all_links, get_num_links
from pybullet_planning.interfaces.env_manager import get_model_info

from .pose_transformation import unit_pose, apply_affine
from .get_data import get_data_type, get_data_extents, get_data_radius, get_data_height, \
    get_data_filename, get_data_scale, get_collision_data, get_data_pose
from .mesh import read_obj

#####################################
# Bounding box

AABB = namedtuple('AABB', ['lower', 'upper'])

def aabb_from_points(points):
    return AABB(np.min(points, axis=0), np.max(points, axis=0))

def aabb_union(aabbs):
    return aabb_from_points(np.vstack([aabb for aabb in aabbs]))

def aabb_overlap(aabb1, aabb2):
    lower1, upper1 = aabb1
    lower2, upper2 = aabb2
    return np.less_equal(lower1, upper2).all() and \
           np.less_equal(lower2, upper1).all()

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

def vertices_from_data(data):
    geometry_type = get_data_type(data)
    #if geometry_type == p.GEOM_SPHERE:
    #    parameters = [get_data_radius(data)]
    if geometry_type == p.GEOM_BOX:
        extents = np.array(get_data_extents(data))
        aabb = AABB(-extents/2., +extents/2.)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        # TODO: p.URDF_USE_IMPLICIT_CYLINDER
        radius, height = get_data_radius(data), get_data_height(data)
        extents = np.array([2*radius, 2*radius, height])
        aabb = AABB(-extents/2., +extents/2.)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type == p.GEOM_SPHERE:
        radius = get_data_radius(data)
        half_extents = radius*np.ones(3)
        aabb = AABB(-half_extents, +half_extents)
        vertices = get_aabb_vertices(aabb)
    elif geometry_type == p.GEOM_MESH:
        filename, scale = get_data_filename(data), get_data_scale(data)
        if filename == UNKNOWN_FILE:
            raise RuntimeError(filename)
        mesh = read_obj(filename, decompose=False)
        vertices = [scale*np.array(vertex) for vertex in mesh.vertices]
        # TODO: could compute AABB here for improved speed at the cost of being conservative
    #elif geometry_type == p.GEOM_PLANE:
    #   parameters = [get_data_extents(data)]
    else:
        raise NotImplementedError(geometry_type)
    return apply_affine(get_data_pose(data), vertices)
