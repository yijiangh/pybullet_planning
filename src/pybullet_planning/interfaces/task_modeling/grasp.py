import warnings
from collections import namedtuple
import pybullet as p

from pybullet_planning.interfaces.env_manager.pose_transformation import multiply, invert, set_pose, get_pose
from pybullet_planning.interfaces.robots.link import get_link_subtree, get_link_pose

#####################################
# Grasps

class Attachment(object):
    def __init__(self, parent, parent_link, grasp_pose, child):
        self.parent = parent
        self.parent_link = parent_link
        self.grasp_pose = grasp_pose
        self.child = child
        #self.child_link = child_link # child_link=BASE_LINK
    @property
    def bodies(self):
        from pybullet_planning.interfaces.robots.collision import flatten_links
        return flatten_links(self.child) | flatten_links(self.parent, get_link_subtree(
            self.parent, self.parent_link))
    def assign(self):
        parent_link_pose = get_link_pose(self.parent, self.parent_link)
        child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
        set_pose(self.child, child_pose)
        return child_pose
    def apply_mapping(self, mapping):
        self.parent = mapping.get(self.parent, self.parent)
        self.child = mapping.get(self.child, self.child)
    def to_data(self):
        from pybullet_planning.interfaces.robots.body import get_body_name, get_name
        from pybullet_planning.interfaces.robots.link import get_link_name
        data = {}
        data['parent_name'] = get_body_name(self.parent)
        data['parent_link_name'] = get_link_name(self.parent, self.parent_link)
        data['grasp_pose'] = self.grasp_pose
        child_name =  get_body_name(self.child)
        data['child_name'] = get_name(self.child) if child_name == '' else child_name
        return data
    @classmethod
    def from_data(cls, data, parent=None, child=None):
        from pybullet_planning.interfaces.env_manager.simulation import is_connected
        from pybullet_planning.interfaces.robots.body import body_from_name, get_bodies
        from pybullet_planning.interfaces.robots.link import link_from_name
        if parent is None:
            parent = body_from_name(data['parent_name'])
        else:
            assert parent in get_bodies()
        if child is None:
            child = body_from_name(data['child_name'])
        else:
            assert child in get_bodies()
        parent_link = link_from_name(parent, data['parent_link_name'])
        grasp_pose = data['grasp_pose']
        return cls(parent, parent_link, grasp_pose, child)

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)


def create_attachment(parent, parent_link, child):
    """create an Attachment between the parent body's parent_link and child body, based on their **current pose**

    Parameters
    ----------
    parent : int
        parent body's pb index
    parent_link : int
        parent body's attach link index
    child : [type]
        child body's pb index

    Returns
    -------
    Attachment
    """
    parent_link_pose = get_link_pose(parent, parent_link)
    child_pose = get_pose(child)
    grasp_pose = multiply(invert(parent_link_pose), child_pose)
    return Attachment(parent, parent_link, grasp_pose, child)

######################################################################

GraspInfo = namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])

def body_from_end_effector(end_effector_pose, grasp_pose):
    """get world_from_brick pose from a given world_from_gripper and gripper_from_brick

    Parameters
    ----------
    end_effector_pose : [type]
        world_from_gripper
    grasp_pose : [type]
        gripper_from_brick

    Returns
    -------
    Pose
        world_from_brick
    """

    return multiply(end_effector_pose, grasp_pose)


def end_effector_from_body(body_pose, grasp_pose):
    """get world_from_gripper from the target brick's pose and the grasp pose

        world_from_child * (parent_from_child)^(-1) = world_from_parent
        (parent: gripper, child: body to be grasped)
        world_from_gripper = world_from_block * block_from_gripper
                           = world_from_block * invert(gripper_from_block)

    Parameters
    ----------
    body_pose : Pose
        world_from_body
    grasp_pose : Pose
        gripper_from_body, the body's pose in gripper's frame

    Returns
    -------
    Pose
        world_from_gripper
    """
    return multiply(body_pose, invert(grasp_pose))

def approach_from_grasp(approach_pose, end_effector_pose):
    """get approach_from_brick

    Parameters
    ----------
    approach_pose : [type]
        approach_from_gripper
    end_effector_pose : [type]
        gripper_from_brick

    Returns
    -------
    [type]
        approach_from_brick
    """
    return multiply(approach_pose, end_effector_pose)

def get_grasp_pose(constraint):
    """get grasp poses from a constraint

    Parameters
    ----------
    constraint : [type]
        [description]

    Returns
    -------
    [type]
        [description]
    """
    from pybullet_planning.interfaces.task_modeling.constraint import get_constraint_info
    constraint_info = get_constraint_info(constraint)
    assert(constraint_info.constraintType == p.JOINT_FIXED)
    joint_from_parent = (constraint_info.jointPivotInParent, constraint_info.jointFrameOrientationParent)
    joint_from_child = (constraint_info.jointPivotInChild, constraint_info.jointFrameOrientationChild)
    return multiply(invert(joint_from_parent), joint_from_child)
