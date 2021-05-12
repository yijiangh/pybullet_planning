from random import randint, random
from .utils import INF, elapsed_time, irange, waypoints_from_path, \
    convex_combination, flatten, compute_path_cost, default_selector
from pybullet_planning.utils.iter_utils import get_pairs
from pybullet_planning.interfaces.env_manager.pose_transformation import get_distance

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

def smooth_path(path, extend_fn, collision_fn, distance_fn=None, iterations=50, max_time=INF, verbose=False):
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
    # TODO: makes an assumption on the distance_fn metric
    # TODO: smooth until convergence
    assert (iterations < INF) or (max_time < INF)
    start_time = time.time()
    if distance_fn is None:
        distance_fn = get_distance
    waypoints = waypoints_from_path(path)
    for iteration in irange(iterations):
        #waypoints = waypoints_from_path(waypoints)
        if (elapsed_time(start_time) > max_time) or (len(waypoints) <= 2):
            break
        # TODO: smoothing in the same linear segment when circular

        indices = list(range(len(waypoints)))
        segments = list(get_pairs(indices))
        distances = [distance_fn(waypoints[i], waypoints[j]) for i, j in segments]
        total_distance = sum(distances)
        if verbose:
            print('Iteration: {} | Waypoints: {} | Distance: {:.3f} | Time: {:.3f}'.format(
                iteration, len(waypoints), total_distance, elapsed_time(start_time)))
        # longer segment has a larger probability to be chosen
        probabilities = np.array(distances) / total_distance

        #segment1, segment2 = choices(segments, weights=probabilities, k=2)
        seg_indices = list(range(len(segments)))
        seg_idx1, seg_idx2 = np.random.choice(seg_indices, size=2, replace=True, p=probabilities)
        if seg_idx1 == seg_idx2:
            continue
        if seg_idx2 < seg_idx1: # choices samples with replacement
            seg_idx1, seg_idx2 = seg_idx2, seg_idx1
        segment1, segment2 = segments[seg_idx1], segments[seg_idx2]
        # TODO: option to sample_fn only adjacent pairs
        point1, point2 = [convex_combination(waypoints[i], waypoints[j], w=random())
                          for i, j in [segment1, segment2]]
        i, _ = segment1
        _, j = segment2
        new_waypoints = waypoints[:i+1] + [point1, point2] + waypoints[j:] # TODO: reuse computation
        if compute_path_cost(new_waypoints, cost_fn=distance_fn) >= total_distance:
            continue
        if all(not collision_fn(q) for q in default_selector(extend_fn(point1, point2))):
            waypoints = new_waypoints
    #return waypoints
    return refine_waypoints(waypoints, extend_fn)

#smooth_path = smooth_path_old
