from random import randint, random, choice
from .utils import INF, elapsed_time, irange, waypoints_from_path, \
    convex_combination, flatten, compute_path_cost, default_selector
from pybullet_planning.utils.iter_utils import get_pairs
from pybullet_planning.interfaces.env_manager.pose_transformation import get_distance
from .primitives import distance_fn_from_extend_fn

import time
import numpy as np

__all__ = [
    'refine_waypoints',
    'smooth_path',
]

def refine_waypoints(waypoints, extend_fn):
    """refine a trajectory using the given extend function.

    Parameters
    ----------
    waypoints : list(list(float))
        input path
    extend_fn : function handle
        Extension function - `extend_fn(q1, q2)->[q', ..., q"]`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_extend_fn` for an example

    Returns
    -------
        refined path
    """
    #if len(waypoints) <= 1:
    #    return waypoints
    return list(flatten(extend_fn(q1, q2) for q1, q2 in get_pairs(waypoints))) # [waypoints[0]] +

def smooth_path(path, extend_fn, collision_fn, distance_fn=None, cost_fn=None, sample_fn=None, max_iterations=50, max_time=INF, converge_time=INF, verbose=False):
    """Perform shortcutting by randomly choosing two segments in the path, check if they result in a shorter path cost, and
    repeat for a given number of iterations. ``default_selecter`` (bisect) is performed upon configurations sampled by
    the ``extension_fn`` on the two shortcutting end points to check collision.

    See also:
    - Geraerts R, Overmars MH. Creating High-quality Paths for Motion Planning. IJRR. 2007;26(8):845-863. doi:10.1177/0278364907079280
    - https://github.com/personalrobotics/or_parabolicsmoother

    Parameters
    ----------
    extend_fn : function handle
        Extension function - ``extend_fn(q1, q2)->[q', ..., q"]``
    collision_fn : function handle
        Collision function - ``collision_fn(q)->bool``
    distance_fn : function handle
        distance function - ``distance_fn(q1, q2)->float``, default to None, which will use
        the default euclidean distance.
    iterations : int
        Maximum number of iterations
    max_time: float
        Maximum runtime
    kwargs:
        Keyword arguments

    Returns
    -------
    list(list(float))
        Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: makes an assumption on extend_fn (to avoid sampling the same segment)
    # TODO: smooth until convergence
    # TODO: dynamic expansion of the nearby graph
    start_time = last_time = time.time()
    if (path is None) or (max_iterations is None):
        return path
    assert (max_iterations < INF) or (max_time < INF)
    start_time = time.time()
    if distance_fn is None:
        distance_fn = distance_fn_from_extend_fn(extend_fn)
    if cost_fn is None:
        cost_fn = distance_fn
    # TODO: use extend_fn to extract waypoints
    waypoints = waypoints_from_path(path, difference_fn=None) # TODO: difference_fn
    cost = compute_path_cost(waypoints, cost_fn=cost_fn)
    #paths = [extend_fn(*pair) for pair in get_pairs(waypoints)] # TODO: update incrementally
    #costs = [cost_fn(*pair) for pair in get_pairs(waypoints)]
    for iteration in irange(max_iterations):
        if (elapsed_time(start_time) > max_time) or (elapsed_time(last_time) > converge_time) or (len(waypoints) <= 2):
            break

        segments = get_pairs(range(len(waypoints)))
        weights = [distance_fn(waypoints[i], waypoints[j]) for i, j in segments]
        paths = [list(extend_fn(*pair)) for pair in get_pairs(waypoints)]
        #weights = [len(paths[i]) for i, j in segments]
        probabilities = np.array(weights) / sum(weights)
        if verbose:
            print('Iteration: {} | Waypoints: {} | Cost: {:.3f} | Elapsed: {:.3f} | Remaining: {:.3f}'.format(
                iteration, len(waypoints), cost, elapsed_time(start_time), max_time-elapsed_time(start_time)))

        #segment1, segment2 = choices(segments, weights=probabilities, k=2)
        seg_indices = list(range(len(segments)))
        seg_idx1, seg_idx2 = np.random.choice(seg_indices, size=2, replace=True, p=probabilities)
        if seg_idx1 == seg_idx2: # TODO: ensure not too far away
            continue
        if seg_idx2 < seg_idx1: # choices samples with replacement
            seg_idx1, seg_idx2 = seg_idx2, seg_idx1
        segment1, segment2 = segments[seg_idx1], segments[seg_idx2]
        # TODO: option to sample_fn only adjacent pairs
        #point1, point2 = [convex_combination(waypoints[i], waypoints[j], w=random())
        #                  for i, j in [segment1, segment2]]
        point1, point2 = [choice(paths[i]) for i, j in [segment1, segment2]]

        i, _ = segment1
        _, j = segment2
        shortcut = [point1, point2]
        # if sample_fn is not None:
        #     shortcut = [point1, sample_fn(), point2]
        #shortcut_paths = [extend_fn(*pair) for pair in get_pairs(waypoints)]
        new_waypoints = waypoints[:i+1] + shortcut + waypoints[j:] # TODO: reuse computation
        new_cost = compute_path_cost(new_waypoints, cost_fn=cost_fn)
        if new_cost >= cost: # TODO: cost must have percent improvement above a threshold
            continue
        if not any(collision_fn(q) for q in default_selector(refine_waypoints(shortcut, extend_fn))):
            waypoints = new_waypoints
            cost = new_cost
            last_time = time.time()
    #return waypoints
    return refine_waypoints(waypoints, extend_fn)
#smooth_path = smooth_path_old
