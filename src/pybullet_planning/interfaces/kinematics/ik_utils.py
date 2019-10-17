import math
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT
from pybullet_planning.interfaces.robots import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits
from pybullet_planning.interfaces.geometry import all_between

def inverse_kinematics_helper(robot, link, target_pose, null_space=None):
    (target_point, target_quat) = target_pose
    assert target_point is not None
    if null_space is not None:
        assert target_quat is not None
        lower, upper, ranges, rest = null_space

        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point,
                                                      lowerLimits=lower, upperLimits=upper, jointRanges=ranges, restPoses=rest,
                                                      physicsClientId=CLIENT)
    elif target_quat is None:
        #ikSolver = p.IK_DLS or p.IK_SDLS
        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point,
                                                      #lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp, jointDamping=jd,
                                                      # solver=ikSolver, maxNumIterations=-1, residualThreshold=-1,
                                                      physicsClientId=CLIENT)
    else:
        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat, physicsClientId=CLIENT)
    if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
        return None
    return kinematic_conf

def is_pose_close(pose, target_pose, pos_tolerance=1e-3, ori_tolerance=1e-3*np.pi):
    (point, quat) = pose
    (target_point, target_quat) = target_pose
    if (target_point is not None) and not np.allclose(point, target_point, atol=pos_tolerance, rtol=0):
        return False
    if (target_quat is not None) and not np.allclose(quat, target_quat, atol=ori_tolerance, rtol=0):
        # TODO: account for quaternion redundancy
        return False
    return True

def inverse_kinematics(robot, link, target_pose, max_iterations=200, custom_limits={}, **kwargs):
    movable_joints = get_movable_joints(robot)
    for iterations in range(max_iterations):
        # TODO: stop is no progress
        # TODO: stop if collision or invalid joint limits
        kinematic_conf = inverse_kinematics_helper(robot, link, target_pose)
        if kinematic_conf is None:
            return None
        set_joint_positions(robot, movable_joints, kinematic_conf)
        if is_pose_close(get_link_pose(robot, link), target_pose, **kwargs):
            break
    else:
        return None
    lower_limits, upper_limits = get_custom_limits(robot, movable_joints, custom_limits)
    if not all_between(lower_limits, kinematic_conf, upper_limits):
        return None
    return kinematic_conf
