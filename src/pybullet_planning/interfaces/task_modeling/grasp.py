
from collections import namedtuple
import pybullet as p

from pybullet_planning.utils import CLIENT, BASE_LINK
from pybullet_planning.interfaces.env_manager.pose_transformation import multiply, invert, set_pose, get_pose
from pybullet_planning.interfaces.robots.link import get_link_subtree, get_link_pose

#####################################
# Grasps

GraspInfo = namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])

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
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)

def create_attachment(parent, parent_link, child):
    parent_link_pose = get_link_pose(parent, parent_link)
    child_pose = get_pose(child)
    grasp_pose = multiply(invert(parent_link_pose), child_pose)
    return Attachment(parent, parent_link, grasp_pose, child)

def body_from_end_effector(end_effector_pose, grasp_pose):
    """
    world_from_parent * parent_from_child = world_from_child
    """
    return multiply(end_effector_pose, grasp_pose)

def end_effector_from_body(body_pose, grasp_pose):
    """
    grasp_pose: the body's pose in gripper's frame
    world_from_child * (parent_from_child)^(-1) = world_from_parent
    (parent: gripper, child: body to be grasped)
    Pose_{world,gripper} = Pose_{world,block}*Pose_{block,gripper}
                         = Pose_{world,block}*(Pose_{gripper,block})^{-1}
    """
    return multiply(body_pose, invert(grasp_pose))

def approach_from_grasp(approach_pose, end_effector_pose):
    return multiply(approach_pose, end_effector_pose)

def get_grasp_pose(constraint):
    """
    Grasps are parent_from_child
    """
    from pybullet_planning.interfaces.task_modeling.constraint import get_constraint_info
    constraint_info = get_constraint_info(constraint)
    assert(constraint_info.constraintType == p.JOINT_FIXED)
    joint_from_parent = (constraint_info.jointPivotInParent, constraint_info.jointFrameOrientationParent)
    joint_from_child = (constraint_info.jointPivotInChild, constraint_info.jointFrameOrientationChild)
    return multiply(invert(joint_from_parent), joint_from_child)
