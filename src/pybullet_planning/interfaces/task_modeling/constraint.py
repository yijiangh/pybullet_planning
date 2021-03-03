
from collections import namedtuple
import pybullet as p

from pybullet_planning.utils import get_client, BASE_LINK

#####################################
# Constraints - applies forces when not satisfied

def get_constraints():
    """
    getConstraintUniqueId will take a serial index in range 0..getNumConstraints,  and reports the constraint unique id.
    Note that the constraint unique ids may not be contiguous, since you may remove constraints.
    """
    return [p.getConstraintUniqueId(i, physicsClientId=get_client())
            for i in range(p.getNumConstraints(physicsClientId=get_client()))]

def remove_constraint(constraint):
    p.removeConstraint(constraint, physicsClientId=get_client())

ConstraintInfo = namedtuple('ConstraintInfo', ['parentBodyUniqueId', 'parentJointIndex',
                                               'childBodyUniqueId', 'childLinkIndex', 'constraintType',
                                               'jointAxis', 'jointPivotInParent', 'jointPivotInChild',
                                               'jointFrameOrientationParent', 'jointFrameOrientationChild', 'maxAppliedForce'])

def get_constraint_info(constraint): # getConstraintState
    # TODO: four additional arguments
    return ConstraintInfo(*p.getConstraintInfo(constraint, physicsClientId=get_client())[:11])

def get_fixed_constraints():
    fixed_constraints = []
    for constraint in get_constraints():
        constraint_info = get_constraint_info(constraint)
        if constraint_info.constraintType == p.JOINT_FIXED:
            fixed_constraints.append(constraint)
    return fixed_constraints

def add_fixed_constraint(body, robot, robot_link, max_force=None):
    from pybullet_planning.interfaces.env_manager.pose_transformation import get_pose, unit_point, unit_quat, multiply, invert
    from pybullet_planning.interfaces.robots import get_com_pose

    body_link = BASE_LINK
    body_pose = get_pose(body)
    #body_pose = get_com_pose(body, link=body_link)
    #end_effector_pose = get_link_pose(robot, robot_link)
    end_effector_pose = get_com_pose(robot, robot_link)
    grasp_pose = multiply(invert(end_effector_pose), body_pose)
    point, quat = grasp_pose
    # TODO: can I do this when I'm not adjacent?
    # joint axis in local frame (ignored for JOINT_FIXED)
    #return p.createConstraint(robot, robot_link, body, body_link,
    #                          p.JOINT_FIXED, jointAxis=unit_point(),
    #                          parentFramePosition=unit_point(),
    #                          childFramePosition=point,
    #                          parentFrameOrientation=unit_quat(),
    #                          childFrameOrientation=quat)
    constraint = p.createConstraint(robot, robot_link, body, body_link,  # Both seem to work
                                    p.JOINT_FIXED, jointAxis=unit_point(),
                                    parentFramePosition=point,
                                    childFramePosition=unit_point(),
                                    parentFrameOrientation=quat,
                                    childFrameOrientation=unit_quat(),
                                    physicsClientId=get_client())
    if max_force is not None:
        p.changeConstraint(constraint, maxForce=max_force, physicsClientId=get_client())
    return constraint

def remove_fixed_constraint(body, robot, robot_link):
    for constraint in get_fixed_constraints():
        constraint_info = get_constraint_info(constraint)
        if (body == constraint_info.childBodyUniqueId) and \
                (BASE_LINK == constraint_info.childLinkIndex) and \
                (robot == constraint_info.parentBodyUniqueId) and \
                (robot_link == constraint_info.parentJointIndex):
            remove_constraint(constraint)
