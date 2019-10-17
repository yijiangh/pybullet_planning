from itertools import product
from collections import defaultdict, deque, namedtuple
import pybullet as p

from pybullet_planning.utils import BASE_LINK, CLIENT
from pybullet_planning.interfaces import multiply, invert
from .joint import get_num_joints, get_joints, get_joint_info, is_movable, prune_fixed_joints
from .body import get_base_name, get_pose

#####################################
# Links

get_num_links = get_num_joints
get_links = get_joints # Does not include BASE_LINK

def child_link_from_joint(joint):
    return joint # link

def parent_joint_from_link(link):
    return link # joint

def get_all_links(body):
    return [BASE_LINK] + list(get_links(body))

def get_link_name(body, link):
    if link == BASE_LINK:
        return get_base_name(body)
    return get_joint_info(body, link).linkName.decode('UTF-8')

def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex

parent_link_from_joint = get_link_parent

def link_from_name(body, name):
    if name == get_base_name(body):
        return BASE_LINK
    for link in get_joints(body):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)


def has_link(body, name):
    try:
        link_from_name(body, name)
    except ValueError:
        return False
    return True

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

def get_link_state(body, link, kinematics=True, velocity=True):
    # TODO: the defaults are set to False?
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/pybullet.c
    return LinkState(*p.getLinkState(body, link, #computeLinkVelocity=velocity, computeForwardKinematics=kinematics,
                                     physicsClientId=CLIENT))

def get_com_pose(body, link): # COM = center of mass
    link_state = get_link_state(body, link)
    return link_state.linkWorldPosition, link_state.linkWorldOrientation

def get_link_inertial_pose(body, link):
    link_state = get_link_state(body, link)
    return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation

def get_link_pose(body, link):
    if link == BASE_LINK:
        return get_pose(body)
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    link_state = get_link_state(body, link) #, kinematics=True, velocity=False)
    return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

def get_relative_pose(body, link1, link2=BASE_LINK):
    world_from_link1 = get_link_pose(body, link1)
    world_from_link2 = get_link_pose(body, link2)
    link2_from_link1 = multiply(invert(world_from_link2), world_from_link1)
    return link2_from_link1

#####################################

def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}

def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children

def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])

def get_link_ancestors(body, link):
    # Returns in order of depth
    # Does not include link
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]

def get_joint_ancestors(body, joint):
    link = child_link_from_joint(joint)
    return get_link_ancestors(body, link) + [link]

def get_movable_joint_ancestors(body, link):
    return prune_fixed_joints(body, get_joint_ancestors(body, link))

def get_joint_descendants(body, link):
    return list(map(parent_joint_from_link, get_link_descendants(body, link)))

def get_movable_joint_descendants(body, link):
    return prune_fixed_joints(body, get_joint_descendants(body, link))

def get_link_descendants(body, link, test=lambda l: True):
    descendants = []
    for child in get_link_children(body, link):
        if test(child):
            descendants.append(child)
            descendants.extend(get_link_descendants(body, child, test=test))
    return descendants

def get_link_subtree(body, link, **kwargs):
    return [link] + get_link_descendants(body, link, **kwargs)

def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or \
           (get_link_parent(body, link2) == link1)

def get_adjacent_links(body):
    adjacent = set()
    for link in get_links(body):
        parent = get_link_parent(body, link)
        adjacent.add((link, parent))
        #adjacent.add((parent, link))
    return adjacent

def get_adjacent_fixed_links(body):
    return list(filter(lambda item: not is_movable(body, item[0]),
                       get_adjacent_links(body)))

def get_fixed_links(body):
    edges = defaultdict(list)
    for link, parent in get_adjacent_fixed_links(body):
        edges[link].append(parent)
        edges[parent].append(link)
    visited = set()
    fixed = set()
    for initial_link in get_links(body):
        if initial_link in visited:
            continue
        cluster = [initial_link]
        queue = deque([initial_link])
        visited.add(initial_link)
        while queue:
            for next_link in edges[queue.popleft()]:
                if next_link not in visited:
                    cluster.append(next_link)
                    queue.append(next_link)
                    visited.add(next_link)
        fixed.update(product(cluster, cluster))
    return fixed
