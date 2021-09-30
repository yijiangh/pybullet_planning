from itertools import takewhile

from .rrt import TreeNode
from .utils import argmin, negate

ASYMETRIC = True


def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward and ASYMETRIC:
        return reversed(list(extend_fn(q2, q1))) # Forward model
    return extend_fn(q1, q2)


def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap=False, tree_frequency=1):
    """Takes current tree and extend it towards a new node (`target`).
    """
    assert tree_frequency >= 1
    # the nearest node in the tree to the target
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    # the segments by connecting last to the target using the given extend fn
    extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    # check if the extended path collision-free, stop until find a collision
    safe = list(takewhile(negate(collision_fn), extend))
    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success
