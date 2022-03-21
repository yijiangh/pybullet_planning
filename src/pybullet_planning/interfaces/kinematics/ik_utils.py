import math
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT

def inverse_kinematics_helper(robot, link, target_pose, null_space=None):
    (target_point, target_quat) = target_pose
    assert target_point is not None
    try:
        if null_space is not None:
            assert target_quat is not None
            lower, upper, ranges, rest = null_space

            kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat,
                                                          lowerLimits=lower, upperLimits=upper, jointRanges=ranges, restPoses=rest,
                                                          physicsClientId=CLIENT,
                                                        #   maxNumIterations=1000,
                                                        #   residualThreshold=1e-12,
                                                          )
        elif target_quat is None:
            #ikSolver = p.IK_DLS or p.IK_SDLS
            kinematic_conf = p.calculateInverseKinematics(robot, link, target_point,
                                                          #lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp, jointDamping=jd,
                                                          # solver=ikSolver, maxNumIterations=-1, residualThreshold=-1,
                                                          physicsClientId=CLIENT)
        else:
            # ! normal case
            kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat,
                                                          physicsClientId=CLIENT,
                                                        #   maxNumIterations=1000,
                                                        #   residualThreshold=1e-12,
                                                          )
    except p.error as e:
        kinematic_conf = None
    if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
        return None
    return kinematic_conf

def is_pose_close(pose, target_pose, pos_tolerance=1e-3, ori_tolerance=1e-3*np.pi):
    (point, quat) = pose
    (target_point, target_quat) = target_pose
    if (target_point is not None) and not np.allclose(point, target_point, atol=pos_tolerance, rtol=0):
        return False
    if (target_quat is not None) and not (np.allclose(quat, target_quat, atol=ori_tolerance, rtol=0) or \
            np.allclose(quat, -np.array(target_quat), atol=ori_tolerance, rtol=0)):
        return False
    return True

def inverse_kinematics(robot, link, target_pose, max_iterations=200, custom_limits={}, **kwargs):
    from pybullet_planning.interfaces.env_manager.pose_transformation import all_between
    from pybullet_planning.interfaces.robots import get_movable_joints, set_joint_positions, get_link_pose, get_custom_limits

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


def snap_sols(sols, q_guess, joint_limits, weights=None, best_sol_only=False):
    """get the best solution based on closeness to the q_guess and weighted joint diff

    Parameters
    ----------
    sols : [type]
        [description]
    q_guess : [type]
        [description]
    joint_limits : [type]
        [description]
    weights : [type], optional
        [description], by default None
    best_sol_only : bool, optional
        [description], by default False

    Returns
    -------
    lists of joint conf (list)
    or
    joint conf (list)
    """
    import numpy as np
    valid_sols = []
    dof = len(q_guess)
    if not weights:
        weights = [1.0] * dof
    else:
        assert dof == len(weights)

    for sol in sols:
        test_sol = np.ones(dof)*9999.
        for i in range(dof):
            for add_ang in [-2.*np.pi, 0, 2.*np.pi]:
                test_ang = sol[i] + add_ang
                if (test_ang <= joint_limits[i][1] and test_ang >= joint_limits[i][0] and \
                    abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                    test_sol[i] = test_ang
        if np.all(test_sol != 9999.):
            valid_sols.append(test_sol.tolist())

    if len(valid_sols) == 0:
        return []

    if best_sol_only:
        best_sol_ind = np.argmin(np.sum((weights*(valid_sols - np.array(q_guess)))**2,1))
        return valid_sols[best_sol_ind]
    else:
        return valid_sols
