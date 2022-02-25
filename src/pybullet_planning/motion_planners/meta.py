"""unified entry API for calling different planners
"""
import time

from .lattice import lattice
from .lazy_prm import lazy_prm
from .prm import prm
from .rrt import rrt
from .rrt_connect import rrt_connect, birrt
from .rrt_star import rrt_star

from .utils import INF
from .smoothing import smooth_path
from .utils import RRT_RESTARTS, RRT_SMOOTHING, INF, irange, elapsed_time, compute_path_cost, default_selector, get_pairs, \
    remove_redundant

def direct_path(start, goal, extend_fn, collision_fn, sweep_collision_fn=None, **kwargs):
    """direct linear path connnecting start and goal using the extension fn.

    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param sweep_collision_fn (Optional): Sweep collision function - collision_fn(q0, q1)->bool
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: version which checks whether the segment is valid
    if collision_fn(start) or collision_fn(goal):
        return None
    path = list(extend_fn(start, goal))
    if any(collision_fn(q) for q in default_selector(path)):
        return None
    if sweep_collision_fn is not None:
        if any(sweep_collision_fn(q0, q1) for q0, q1 in default_selector(get_pairs(path))):
            return None
    return path

def check_direct(start, goal, extend_fn, collision_fn, **kwargs):
    if any(collision_fn(q) for q in [start, goal]):
        return False
    return direct_path(start, goal, extend_fn, collision_fn, **kwargs)

#################################################################

def random_restarts(solve_fn, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, verbose=False, **kwargs):
    """Apply random restarts to a given planning algorithm to obtain multiple solutions.

    Parameters
    ----------
    solve_fn : function handle
        motion planner function, e.g. ``birrt``
    start : list
        start conf
    goal : list
        end conf
    distance_fn : function handle
        Distance function - `distance_fn(q1, q2)->float`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_difference_fn` for an example
    sample_fn : function handle
        configuration space sampler - `sample_fn()->conf`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_sample_fn` for an example
    extend_fn : function handle
        Extension function - `extend_fn(q1, q2)->[q', ..., q"]`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_extend_fn` for an example
    collision_fn : function handle
        Collision function - `collision_fn(q)->bool`
        see `pybullet_planning.interfaces.robots.collision.get_collision_fn` for an example
    restarts : int, optional
        number of random restarts, by default RRT_RESTARTS
    smooth : int, optional
        smoothing iterations, by default RRT_SMOOTHING
    success_cost : float, optional
        random restarts will terminate early if a path with cost lower than this number is found, by default 0.
    max_time : float, optional
        max allowed runtime, by default INF
    max_solutions : int, optional
        number of solutions wanted, random restarts will terminate early if more solutions are found, by default 1
    verbose : bool, optional
        print toggle, by default False

    Returns
    -------
    list
        list of paths, [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    solutions = []
    path = check_direct(start, goal, extend_fn, collision_fn, **kwargs)
    if path is False:
        return None
    if path is not None:
        solutions.append(path)

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue
        path = smooth_path(path, extend_fn, collision_fn, max_smooth_iterations=smooth,
                           max_time=max_time-elapsed_time(start_time), **kwargs)
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    if verbose:
        print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
            path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions

def solve_and_smooth(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    """plan and smooth without random restarting.
    """
    return random_restarts(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, restarts=0, **kwargs)

#################################################################

def solve_motion_plan(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, algorithm='birrt',
          max_time=INF, max_iterations=INF, num_samples=100, smooth=None, **kwargs):
    # TODO: allow distance_fn to be skipped
    # TODO: return lambda function
    start_time = time.time()
    path = check_direct(start, goal, extend_fn, collision_fn, **kwargs)
    if path is not None:
        return path
    max_time -= elapsed_time(start_time)
    if algorithm == 'prm':
        path = prm(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   num_samples=num_samples, **kwargs)
    elif algorithm == 'lazy_prm':
        path = lazy_prm(start, goal, sample_fn, extend_fn, collision_fn,
                        num_samples=num_samples, max_time=max_time, **kwargs)[0]
    elif algorithm == 'rrt':
        path = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   max_iterations=max_iterations, max_time=max_time, **kwargs)
    elif algorithm == 'rrt_connect':
        path = rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                           max_iterations=max_iterations, max_time=max_time, **kwargs)
    elif algorithm == 'birrt':
        # TODO: checks the straight-line twice
        path = birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                     max_iterations=max_iterations, max_time=max_time, smooth=None, **kwargs) # restarts=2
    elif algorithm == 'rrt_star':
        path = rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, radius=1,
                        max_iterations=max_iterations, max_time=max_time, **kwargs)
    elif algorithm == 'lattice':
        path = lattice(start, goal, extend_fn, collision_fn, distance_fn=distance_fn, max_time=INF, **kwargs)
    else:
        raise NotImplementedError(algorithm)
    if path:
        path = remove_redundant(path)
    return smooth_path(path, extend_fn, collision_fn, max_smooth_iterations=smooth, max_time=max_time-elapsed_time(start_time), **kwargs)
