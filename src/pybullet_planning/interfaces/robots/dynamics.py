
from collections import defaultdict, deque, namedtuple

import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK, STATIC_MASS
from pybullet_planning.interfaces.geometry import Pose, multiply, invert
from .link import get_all_links, parent_link_from_joint
from .joint import get_joint_parent_frame
from .body import get_bodies

#####################################

DynamicsInfo = namedtuple('DynamicsInfo', ['mass', 'lateral_friction',
                                           'local_inertia_diagonal', 'local_inertial_pos',  'local_inertial_orn',
                                           'restitution', 'rolling_friction', 'spinning_friction',
                                           'contact_damping', 'contact_stiffness'])

def get_dynamics_info(body, link=BASE_LINK):
    return DynamicsInfo(*p.getDynamicsInfo(body, link, physicsClientId=CLIENT))

get_link_info = get_dynamics_info

def get_mass(body, link=BASE_LINK):
    # TOOD: get full mass
    return get_dynamics_info(body, link).mass

def set_dynamics(body, link=BASE_LINK, **kwargs):
    # TODO: iterate over all links
    p.changeDynamics(body, link, physicsClientId=CLIENT, **kwargs)

def set_mass(body, mass, link=BASE_LINK):
    set_dynamics(body, link=link, mass=mass)

def set_static(body):
    for link in get_all_links(body):
        set_mass(body, mass=STATIC_MASS, link=link)

# this is messing up the dependencies...
# def set_all_static():
#     # TODO: mass saver
#     disable_gravity()
#     for body in get_bodies():
#         set_static(body)

def get_joint_inertial_pose(body, joint):
    dynamics_info = get_dynamics_info(body, joint)
    return dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn

def get_local_link_pose(body, joint):
    parent_joint = parent_link_from_joint(body, joint)

    #world_child = get_link_pose(body, joint)
    #world_parent = get_link_pose(body, parent_joint)
    ##return multiply(invert(world_parent), world_child)
    #return multiply(world_child, invert(world_parent))

    # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169
    parent_com = get_joint_parent_frame(body, joint)
    tmp_pose = invert(multiply(get_joint_inertial_pose(body, joint), parent_com))
    parent_inertia = get_joint_inertial_pose(body, parent_joint)
    #return multiply(parent_inertia, tmp_pose) # TODO: why is this wrong...
    _, orn = multiply(parent_inertia, tmp_pose)
    pos, _ = multiply(parent_inertia, Pose(parent_com[0]))
    return (pos, orn)
