import math
import random
import numpy as np

from pybullet_planning.interfaces import Pose, Euler, unit_pose, point_from_pose, multiply
from pybullet_planning.interfaces import approximate_as_prism, approximate_as_cylinder
from pybullet_planning.interfaces import get_relative_pose, get_link_subtree, clone_body, set_static, get_link_pose, \
    set_pose

TOOL_POSE = Pose(euler=Euler(pitch=np.pi/2)) # l_gripper_tool_frame (+x out of gripper arm)

GRASP_LENGTH = 0.
MAX_GRASP_WIDTH = np.inf
SIDE_HEIGHT_OFFSET = 0.03 # z distance from top of object

def get_top_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    """[summary]

    Parameters
    ----------
    body : [type]
        [description]
    under : bool, optional
        [description], by default False
    tool_pose : [type], optional
        tool from grasp, by default TOOL_POSE
    body_pose : [type], optional
        [description], by default unit_pose()
    max_width : [type], optional
        [description], by default MAX_GRASP_WIDTH
    grasp_length : [type], optional
        [description], by default GRASP_LENGTH

    Returns
    -------
    [type]
        [description]
    """
    # TODO: rename the box grasps
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    reflect_z = Pose(euler=[0, math.pi, 0])
    translate_z = Pose(point=[0, 0, h / 2 - grasp_length])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, math.pi / 2 + i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = Pose(euler=[0, 0, i * math.pi])
            grasps += [multiply(tool_pose, translate_z, rotate_z,
                                reflect_z, translate_center, body_pose)]
    return grasps

def get_side_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                    max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET):
    """[summary]

    Parameters
    ----------
    body : [type]
        [description]
    under : bool, optional
        [description], by default False
    tool_pose : [type], optional
        [description], by default TOOL_POSE
    body_pose : [type], optional
        [description], by default unit_pose()
    max_width : [type], optional
        [description], by default MAX_GRASP_WIDTH
    grasp_length : [type], optional
        [description], by default GRASP_LENGTH
    top_offset : [type], optional
        [description], by default SIDE_HEIGHT_OFFSET

    Returns
    -------
    [type]
        [description]
    """
    # TODO: compute bounding box width wrt tool frame
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    #x_offset = 0
    x_offset = h/2 - top_offset
    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        if w <= max_width:
            translate_z = Pose(point=[x_offset, 0, l / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[math.pi / 2 + i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([w])
        if l <= max_width:
            translate_z = Pose(point=[x_offset, 0, w / 2 - grasp_length])
            for i in range(2):
                rotate_z = Pose(euler=[i * math.pi, 0, 0])
                grasps += [multiply(tool_pose, translate_z, rotate_z, swap_xz,
                                    translate_center, body_pose)]  # , np.array([l])
    return grasps

def get_side_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET):
    """[summary]

    Parameters
    ----------
    body : [type]
        [description]
    under : bool, optional
        [description], by default False
    tool_pose : [type], optional
        [description], by default TOOL_POSE
    body_pose : [type], optional
        [description], by default unit_pose()
    max_width : [type], optional
        [description], by default MAX_GRASP_WIDTH
    grasp_length : [type], optional
        [description], by default GRASP_LENGTH
    top_offset : [type], optional
        [description], by default SIDE_HEIGHT_OFFSET

    Yields
    -------
    [type]
        [description]
    """
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_center = Pose(point_from_pose(body_pose)-center)
    #x_offset = 0
    x_offset = height/2 - top_offset
    if max_width < diameter:
        return
    while True:
        theta = random.uniform(0, 2*np.pi)
        translate_rotate = ([x_offset, 0, diameter / 2 - grasp_length], quat_from_euler([theta, 0, 0]))
        for j in range(1 + under):
            swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
            yield multiply(tool_pose, translate_rotate, swap_xz, translate_center, body_pose)

def get_edge_cylinder_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                             grasp_length=GRASP_LENGTH):
    """[summary]

    Parameters
    ----------
    body : [type]
        [description]
    under : bool, optional
        [description], by default False
    tool_pose : [type], optional
        [description], by default TOOL_POSE
    body_pose : [type], optional
        [description], by default unit_pose()
    grasp_length : [type], optional
        [description], by default GRASP_LENGTH

    Yields
    -------
    [type]
        [description]
    """
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    translate_yz = Pose(point=[0, diameter/2, height/2 - grasp_length])
    reflect_y = Pose(euler=[0, math.pi, 0])
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    while True:
        theta = random.uniform(0, 2*np.pi)
        rotate_z = Pose(euler=[0, 0, theta])
        for i in range(1 + under):
            rotate_under = Pose(euler=[0, 0, i * math.pi])
            yield multiply(tool_pose, rotate_under, translate_yz, rotate_z,
                           reflect_y, translate_center, body_pose)

##############################################

class EndEffector(object):
    """a convenient class for creating and manipulating an end effector

    Note: the end effector needs to be modeled in the robot's URDF.

    """
    def __init__(self, robot, ee_link, tool_link, **kwargs):
        self.robot = robot
        self.ee_link = ee_link
        self.tool_link = tool_link
        self.tool_from_ee = get_relative_pose(self.robot, self.ee_link, self.tool_link)
        tool_links = get_link_subtree(robot, self.ee_link)
        self.body = clone_body(robot, links=tool_links, **kwargs)
        set_static(self.body)
        # for link in get_all_links(tool_body):
        #    set_color(tool_body, np.zeros(4), link)
    def get_tool_pose(self):
        return get_link_pose(self.robot, self.tool_link)
    def set_pose(self, tool_pose):
        pose = multiply(tool_pose, self.tool_from_ee)
        set_pose(self.body, pose)
        return pose
    @property
    def tool_from_root(self):
        return self.tool_from_ee
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.robot, self.body)
