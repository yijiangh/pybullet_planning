from collections import namedtuple
import pybullet as p

from pybullet_planning.utils import get_client, CIRCULAR_LIMITS, UNBOUNDED_LIMITS, INF

#####################################
# Joints

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute', # 0
    p.JOINT_PRISMATIC: 'prismatic', # 1
    p.JOINT_SPHERICAL: 'spherical', # 2
    p.JOINT_PLANAR: 'planar', # 3
    p.JOINT_FIXED: 'fixed', # 4
    p.JOINT_POINT2POINT: 'point2point', # 5
    p.JOINT_GEAR: 'gear', # 6
}

def get_num_joints(body):
    return p.getNumJoints(body, physicsClientId=get_client())

def get_joints(body):
    return list(range(get_num_joints(body)))

def get_joint(body, joint_or_name):
    if type(joint_or_name) is str:
        return joint_from_name(body, joint_or_name)
    return joint_or_name

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])

def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint, physicsClientId=get_client()))

def get_joint_name(body, joint):
    return get_joint_info(body, joint).jointName # .decode('UTF-8')

def get_joint_names(body, joints):
    return [get_joint_name(body, joint) for joint in joints]

def joint_from_name(body, name):
    for joint in get_joints(body):
        jt_name = get_joint_name(body, joint)
        if jt_name == name or jt_name.decode('UTF-8') == name:
            return joint
    raise ValueError(body, name)

def has_joint(body, name):
    try:
        joint_from_name(body, name)
    except ValueError:
        return False
    return True

def joints_from_names(body, names):
    return tuple(joint_from_name(body, name) for name in names)

JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                       'jointReactionForces', 'appliedJointMotorTorque'])

def get_joint_state(body, joint):
    return JointState(*p.getJointState(body, joint, physicsClientId=get_client()))

def get_joint_position(body, joint):
    return get_joint_state(body, joint).jointPosition

def get_joint_velocity(body, joint):
    return get_joint_state(body, joint).jointVelocity

def get_joint_reaction_force(body, joint):
    return get_joint_state(body, joint).jointReactionForces

def get_joint_torque(body, joint):
    return get_joint_state(body, joint).appliedJointMotorTorque

def get_joint_positions(body, joints): # joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)

def get_joint_velocities(body, joints):
    return tuple(get_joint_velocity(body, joint) for joint in joints)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value, targetVelocity=0, physicsClientId=get_client())

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)

def get_configuration(body):
    return get_joint_positions(body, get_movable_joints(body))

def set_configuration(body, values):
    set_joint_positions(body, get_movable_joints(body), values)

def get_full_configuration(body):
    # Cannot alter fixed joints
    return get_joint_positions(body, get_joints(body))

def get_labeled_configuration(body):
    movable_joints = get_movable_joints(body)
    return dict(zip(get_joint_names(body, movable_joints),
                    get_joint_positions(body, movable_joints)))

def get_joint_type(body, joint):
    return get_joint_info(body, joint).jointType

def is_fixed(body, joint):
    return get_joint_type(body, joint) == p.JOINT_FIXED

def is_movable(body, joint):
    return not is_fixed(body, joint)

def prune_fixed_joints(body, joints):
    return [joint for joint in joints if is_movable(body, joint)]

def get_movable_joints(body): # 45 / 87 on pr2
    return prune_fixed_joints(body, get_joints(body))

def joint_from_movable(body, index):
    return get_joints(body)[index]

def movable_from_joints(body, joints):
    movable_from_original = {o: m for m, o in enumerate(get_movable_joints(body))}
    return [movable_from_original[joint] for joint in joints]

def is_circular(body, joint):
    joint_info = get_joint_info(body, joint)
    if joint_info.jointType == p.JOINT_FIXED:
        return False
    return joint_info.jointUpperLimit < joint_info.jointLowerLimit

def get_joint_limits(body, joint):
    # TODO: make a version for several joints?
    if is_circular(body, joint):
        # TODO: return UNBOUNDED_LIMITS
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return joint_info.jointLowerLimit, joint_info.jointUpperLimit

def get_min_limit(body, joint):
    # TODO: rename to min_position
    return get_joint_limits(body, joint)[0]

def get_min_limits(body, joints):
    return [get_min_limit(body, joint) for joint in joints]

def get_max_limit(body, joint):
    return get_joint_limits(body, joint)[1]

def get_max_limits(body, joints):
    return [get_max_limit(body, joint) for joint in joints]

def get_max_velocity(body, joint):
    return get_joint_info(body, joint).jointMaxVelocity

def get_max_force(body, joint):
    return get_joint_info(body, joint).jointMaxForce

def get_joint_q_index(body, joint):
    return get_joint_info(body, joint).qIndex

def get_joint_v_index(body, joint):
    return get_joint_info(body, joint).uIndex

def get_joint_axis(body, joint):
    return get_joint_info(body, joint).jointAxis

def get_joint_parent_frame(body, joint):
    joint_info = get_joint_info(body, joint)
    return joint_info.parentFramePos, joint_info.parentFrameOrn

def violates_limit(body, joint, value):
    if is_circular(body, joint):
        return False
    lower, upper = get_joint_limits(body, joint)
    return (value < lower) or (upper < value)

def violates_limits(body, joints, values):
    return any(violates_limit(body, joint, value) for joint, value in zip(joints, values))

def wrap_position(body, joint, position):
    from pybullet_planning.interfaces.env_manager.pose_transformation import wrap_angle
    if is_circular(body, joint):
        return wrap_angle(position)
    return position

def wrap_positions(body, joints, positions):
    assert len(joints) == len(positions)
    return [wrap_position(body, joint, position)
            for joint, position in zip(joints, positions)]

def get_custom_limits(body, joints, custom_limits={}, circular_limits=UNBOUNDED_LIMITS):
    joint_limits = []
    for joint in joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        elif is_circular(body, joint):
            joint_limits.append(circular_limits)
        else:
            joint_limits.append(get_joint_limits(body, joint))
    return zip(*joint_limits)

def get_custom_max_velocity(body, joints, custom_vel_limits={}):
    vel_limits = []
    for joint in joints:
        if joint in custom_vel_limits:
            vel_limits.append(custom_vel_limits[joint])
        else:
            vel_limits.append(get_max_velocity(body, joint))
    return vel_limits
