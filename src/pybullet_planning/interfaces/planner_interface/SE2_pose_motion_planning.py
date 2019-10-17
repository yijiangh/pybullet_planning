import numpy as np

from pybullet_planning.utils import MAX_DISTANCE, CIRCULAR_LIMITS
from pybullet_planning.interfaces.geometry import circular_difference, pairwise_collision
from pybullet_planning.motion_planners import birrt, direct_path

from pybullet_planning.interfaces.robots import set_base_values, get_base_values

#####################################
# SE(2) pose motion planning

def get_base_difference_fn():
    def fn(q2, q1):
        dx, dy = np.array(q2[:2]) - np.array(q1[:2])
        dtheta = circular_difference(q2[2], q1[2])
        return (dx, dy, dtheta)
    return fn

def get_base_distance_fn(weights=1*np.ones(3)):
    difference_fn = get_base_difference_fn()
    def fn(q1, q2):
        difference = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, difference * difference))
    return fn

def plan_base_motion(body, end_conf, base_limits, obstacles=[], direct=False,
                     weights=1*np.ones(3), resolutions=0.05*np.ones(3),
                     max_distance=MAX_DISTANCE, **kwargs):
    def sample_fn():
        x, y = np.random.uniform(*base_limits)
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        return (x, y, theta)


    difference_fn = get_base_difference_fn()
    distance_fn = get_base_distance_fn(weights=weights)

    def extend_fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        n = int(np.max(steps)) + 1
        q = q1
        for i in range(n):
            q = tuple((1. / (n - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints

    def collision_fn(q):
        # TODO: update this function
        set_base_values(body, q)
        return any(pairwise_collision(body, obs, max_distance=max_distance) for obs in obstacles)

    start_conf = get_base_values(body)
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return None
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return None
    if direct:
        return direct_path(start_conf, end_conf, extend_fn, collision_fn)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)
