import numpy as np
import pybullet as p

from pybullet_planning.utils import CIRCULAR_LIMITS
from pybullet_planning.interfaces.geometry import Euler, Pose
from pybullet_planning.interfaces.geometry import get_center_extent, get_aabb, aabb_contains_aabb, aabb2d_from_aabb, \
    aabb_contains_point, unit_pose, multiply
from pybullet_planning.interfaces.robots import get_link_subtree, get_link_pose, set_pose, get_pose, get_point

#####################################
# Placements

def stable_z_on_aabb(body, aabb):
    center, extent = get_center_extent(body)
    _, upper = aabb
    return (upper + extent/2 + (get_point(body) - center))[2]

def stable_z(body, surface, surface_link=None):
    return stable_z_on_aabb(body, get_aabb(surface, link=surface_link))

def is_placed_on_aabb(body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    assert (0 <= above_epsilon) and (0 <= below_epsilon)
    top_aabb = get_aabb(body) # TODO: approximate_as_prism
    top_z_min = top_aabb[0][2]
    bottom_z_max = bottom_aabb[1][2]
    return ((bottom_z_max - below_epsilon) <= top_z_min <= (bottom_z_max + above_epsilon)) and \
           (aabb_contains_aabb(aabb2d_from_aabb(top_aabb), aabb2d_from_aabb(bottom_aabb)))

def is_placement(body, surface, **kwargs):
    return is_placed_on_aabb(body, get_aabb(surface), **kwargs)

def is_center_on_aabb(body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    # TODO: compute AABB in origin
    # TODO: use center of mass?
    assert (0 <= above_epsilon) and (0 <= below_epsilon)
    center, extent = get_center_extent(body) # TODO: approximate_as_prism
    base_center = center - np.array([0, 0, extent[2]])/2
    top_z_min = base_center[2]
    bottom_z_max = bottom_aabb[1][2]
    return ((bottom_z_max - abs(below_epsilon)) <= top_z_min <= (bottom_z_max + abs(above_epsilon))) and \
           (aabb_contains_point(base_center[:2], aabb2d_from_aabb(bottom_aabb)))

def is_center_stable(body, surface, **kwargs):
    return is_center_on_aabb(body, get_aabb(surface), **kwargs)

def sample_placement_on_aabb(top_body, bottom_aabb, top_pose=unit_pose(),
                             percent=1.0, max_attempts=50, epsilon=1e-3):
    # TODO: transform into the coordinate system of the bottom
    # TODO: maybe I should instead just require that already in correct frame
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = Euler(yaw=theta)
        set_pose(top_body, multiply(Pose(euler=rotation), top_pose))
        center, extent = get_center_extent(top_body)
        lower = (np.array(bottom_aabb[0]) + percent*extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent*extent/2)[:2]
        if np.less(upper, lower).any():
            continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2] + epsilon
        point = np.array([x, y, z]) + (get_point(top_body) - center)
        pose = multiply(Pose(point, rotation), top_pose)
        set_pose(top_body, pose)
        return pose
    return None

def sample_placement(top_body, bottom_body, bottom_link=None, **kwargs):
    bottom_aabb = get_aabb(bottom_body, link=bottom_link)
    return sample_placement_on_aabb(top_body, bottom_aabb, **kwargs)
