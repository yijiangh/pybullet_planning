import time

from itertools import takewhile
from .meta import direct_path, random_restarts
from .rrt import TreeNode, configs
from .utils import irange, argmin, RRT_ITERATIONS, RRT_RESTARTS, RRT_SMOOTHING, INF, negate, elapsed_time

__all__ = [
    'rrt_connect',
    'birrt',
    'direct_path',
    ]

ASYMETRIC = True

def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward and ASYMETRIC:
        return reversed(list(extend_fn(q2, q1))) # Forward model
    return extend_fn(q1, q2)

def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap=False, tree_frequency=1):
    assert tree_frequency >= 1
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    safe = list(takewhile(negate(collision_fn), extend))
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success

def rrt_connect(q1, q2, distance_fn, sample_fn, extend_fn, collision_fn,
                iterations=RRT_ITERATIONS, tree_frequency=1, max_time=INF, **kwargs):
    """RRT connect algorithm: http://www.kuffner.org/james/papers/kuffner_icra2000.pdf

    Parameters
    ----------
    q1 : list
        start configuration
    q2 : list
        end configuration
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
    iterations : int, optional
        rrt iterations, by default RRT_ITERATIONS
    tree_frequency : int, optional
        The frequency of adding tree nodes when extending.
        For example, if tree_freq=2, then a tree node is added every three nodes, by default 1
    max_time : float, optional
        maximal allowed runtime, by default INF

    Returns
    -------
    list(list(float))
        the computed path, i.e. a list of configurations
        return None if no plan is found.
    """
    start_time = time.time()
    assert tree_frequency >= 1
    if collision_fn(q1) or collision_fn(q2):
        return None
    nodes1, nodes2 = [TreeNode(q1)], [TreeNode(q2)]
    for iteration in irange(iterations):
        if max_time <= elapsed_time(start_time):
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        target = sample_fn()
        last1, _ = extend_towards(tree1, target, distance_fn, extend_fn, collision_fn,
                                  swap, **kwargs)
        last2, success = extend_towards(tree2, last1.config, distance_fn, extend_fn, collision_fn,
                                        not swap, **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            # print('RRT connect: {} iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            return configs(path1[:-1] + path2[::-1])
    return None


def birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    solutions = random_restarts(rrt_connect, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]
