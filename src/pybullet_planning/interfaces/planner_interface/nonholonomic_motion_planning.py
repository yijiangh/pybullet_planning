import numpy as np

from pybullet_planning.utils import PI, MAX_DISTANCE
from pybullet_planning.interfaces.geometry import get_angle
from pybullet_planning.interfaces.robots import get_joint_positions
from pybullet_planning.motion_planners import birrt

from .joint_motion_planning import get_distance_fn, get_extend_fn, get_sample_fn, get_collision_fn, check_initial_end
#####################################

def get_closest_angle_fn(body, joints, linear_weight=1., angular_weight=1., reversible=True):
    assert len(joints) == 3
    linear_extend_fn = get_distance_fn(body, joints[:2], weights=linear_weight*np.ones(2))
    angular_extend_fn = get_distance_fn(body, joints[2:], weights=[angular_weight])

    def closest_fn(q1, q2):
        angle_and_distance = []
        for direction in [0, PI] if reversible else [PI]:
            angle = get_angle(q1[:2], q2[:2]) + direction
            distance = angular_extend_fn(q1[2:], [angle]) \
                       + linear_extend_fn(q1[:2], q2[:2]) \
                       + angular_extend_fn([angle], q2[2:])
            angle_and_distance.append((angle, distance))
        return min(angle_and_distance, key=lambda pair: pair[1])
    return closest_fn

def get_nonholonomic_distance_fn(body, joints, weights=None, **kwargs):
    assert weights is None
    closest_angle_fn = get_closest_angle_fn(body, joints, **kwargs)

    def distance_fn(q1, q2):
        _, distance = closest_angle_fn(q1, q2)
        return distance
    return distance_fn

def get_nonholonomic_extend_fn(body, joints, resolutions=None, **kwargs):
    assert resolutions is None
    assert len(joints) == 3
    linear_extend_fn = get_extend_fn(body, joints[:2])
    angular_extend_fn = get_extend_fn(body, joints[2:])
    closest_angle_fn = get_closest_angle_fn(body, joints, **kwargs)

    def extend_fn(q1, q2):
        angle, _ = closest_angle_fn(q1, q2)
        path = []
        for aq in angular_extend_fn(q1[2:], [angle]):
            path.append(np.append(q1[:2], aq))
        for lq in linear_extend_fn(q1[:2], q2[:2]):
            path.append(np.append(lq, [angle]))
        for aq in angular_extend_fn([angle], q2[2:]):
            path.append(np.append(q2[:2], aq))
        return path
    return extend_fn

def plan_nonholonomic_motion(body, joints, end_conf, obstacles=[], attachments=[],
                             self_collisions=True, disabled_collisions=set(),
                             weights=None, resolutions=None, reversible=True,
                             max_distance=MAX_DISTANCE, custom_limits={}, **kwargs):

    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = get_nonholonomic_distance_fn(body, joints, weights=weights, reversible=reversible)
    extend_fn = get_nonholonomic_extend_fn(body, joints, resolutions=resolutions, reversible=reversible)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments,
                                    self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)

    start_conf = get_joint_positions(body, joints)
    if not check_initial_end(start_conf, end_conf, collision_fn):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)
