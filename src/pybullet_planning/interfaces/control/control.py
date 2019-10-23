
import numpy as np
import pybullet as p

from pybullet_planning.utils import CLIENT
from pybullet_planning.interfaces.env_manager.pose_transformation import unit_point
from pybullet_planning.interfaces.robots.joint import get_max_velocity, get_max_force, get_joint_positions, get_movable_joints, \
    movable_from_joints

#####################################
# Control

def control_joint(body, joint, value):
    return p.setJointMotorControl2(bodyUniqueId=body,
                                   jointIndex=joint,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=value,
                                   targetVelocity=0.0,
                                   maxVelocity=get_max_velocity(body, joint),
                                   force=get_max_force(body, joint),
                                   physicsClientId=CLIENT)

def control_joints(body, joints, positions):
    # TODO: the whole PR2 seems to jitter
    #kp = 1.0
    #kv = 0.3
    #forces = [get_max_force(body, joint) for joint in joints]
    #forces = [5000]*len(joints)
    #forces = [20000]*len(joints)
    return p.setJointMotorControlArray(body, joints, p.POSITION_CONTROL,
                                       targetPositions=positions,
                                       targetVelocities=[0.0] * len(joints),
                                       physicsClientId=CLIENT) #,
                                        #positionGains=[kp] * len(joints),
                                        #velocityGains=[kv] * len(joints),)
                                        #forces=forces)

def joint_controller(body, joints, target, tolerance=1e-3):
    assert(len(joints) == len(target))
    positions = get_joint_positions(body, joints)
    while not np.allclose(positions, target, atol=tolerance, rtol=0):
        control_joints(body, joints, target)
        yield positions
        positions = get_joint_positions(body, joints)

def joint_controller_hold(body, joints, target, **kwargs):
    """
    Keeps other joints in place
    """
    movable_joints = get_movable_joints(body)
    conf = list(get_joint_positions(body, movable_joints))
    for joint, value in zip(movable_from_joints(body, joints), target):
        conf[joint] = value
    return joint_controller(body, movable_joints, conf, **kwargs)

def joint_controller_hold2(body, joints, positions, velocities=None,
                           tolerance=1e-2 * np.pi, position_gain=0.05, velocity_gain=0.01):
    """
    Keeps other joints in place
    """
    # TODO: velocity_gain causes the PR2 to oscillate
    if velocities is None:
        velocities = [0.] * len(positions)
    movable_joints = get_movable_joints(body)
    target_positions = list(get_joint_positions(body, movable_joints))
    target_velocities = [0.] * len(movable_joints)
    movable_from_original = {o: m for m, o in enumerate(movable_joints)}
    #print(list(positions), list(velocities))
    for joint, position, velocity in zip(joints, positions, velocities):
        target_positions[movable_from_original[joint]] = position
        target_velocities[movable_from_original[joint]] = velocity
    # return joint_controller(body, movable_joints, conf)
    current_conf = get_joint_positions(body, movable_joints)
    #forces = [get_max_force(body, joint) for joint in movable_joints]
    while not np.allclose(current_conf, target_positions, atol=tolerance, rtol=0):
        # TODO: only enforce velocity constraints at end
        p.setJointMotorControlArray(body, movable_joints, p.POSITION_CONTROL,
                                    targetPositions=target_positions,
                                    #targetVelocities=target_velocities,
                                    positionGains=[position_gain] * len(movable_joints),
                                    #velocityGains=[velocity_gain] * len(movable_joints),
                                    #forces=forces,
                                    physicsClientId=CLIENT)
        yield current_conf
        current_conf = get_joint_positions(body, movable_joints)

def trajectory_controller(body, joints, path, **kwargs):
    for target in path:
        for positions in joint_controller(body, joints, target, **kwargs):
            yield positions

def simulate_controller(controller, max_time=np.inf): # Allow option to sleep rather than yield?
    from pybullet_planning.interfaces.env_manager import get_time_step, step_simulation
    sim_dt = get_time_step()
    sim_time = 0.0
    for _ in controller:
        if max_time < sim_time:
            break
        step_simulation()
        sim_time += sim_dt
        yield sim_time

def velocity_control_joints(body, joints, velocities):
    #kv = 0.3
    return p.setJointMotorControlArray(body, joints, p.VELOCITY_CONTROL,
                                       targetVelocities=velocities,
                                       physicsClientId=CLIENT) #,
                                        #velocityGains=[kv] * len(joints),)
                                        #forces=forces)

#####################################

def compute_jacobian(robot, link, positions=None):
    joints = get_movable_joints(robot)
    if positions is None:
        positions = get_joint_positions(robot, joints)
    assert len(joints) == len(positions)
    velocities = [0.0] * len(positions)
    accelerations = [0.0] * len(positions)
    translate, rotate = p.calculateJacobian(robot, link, unit_point(), positions,
                                            velocities, accelerations, physicsClientId=CLIENT)
    #movable_from_joints(robot, joints)
    return list(zip(*translate)), list(zip(*rotate)) # len(joints) x 3


def compute_joint_weights(robot, num=100):
    import time
    from pybullet_planning.interfaces.planner_interface.joint_motion_planning import get_sample_fn
    from pybullet_planning.interfaces.robots.link import get_links
    from pybullet_planning.interfaces.robots.dynamics import get_mass

    # http://openrave.org/docs/0.6.6/_modules/openravepy/databases/linkstatistics/#LinkStatisticsModel
    # TODO: use velocities instead
    start_time = time.time()
    joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, joints)
    weighted_jacobian = np.zeros(len(joints))
    links = list(get_links(robot))
    # links = {l for j in joints for l in get_link_descendants(self.robot, j)}
    masses = [get_mass(robot, link) for link in links]  # Volume, AABB volume
    total_mass = sum(masses)
    for _ in range(num):
        conf = sample_fn()
        for mass, link in zip(masses, links):
            translate, rotate = compute_jacobian(robot, link, conf)
            weighted_jacobian += np.array([mass * np.linalg.norm(vec) for vec in translate]) / total_mass
    weighted_jacobian /= num
    print(list(weighted_jacobian))
    print(time.time() - start_time)
    return weighted_jacobian

#####################################
