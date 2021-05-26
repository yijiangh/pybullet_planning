import os
import warnings
from itertools import tee
from copy import copy
from collections import namedtuple
import numpy as np
import pybullet as p

from pybullet_planning.interfaces.env_manager.pose_transformation import get_distance
from pybullet_planning.utils import MAX_DISTANCE, EPS, INF
from pybullet_planning.interfaces.robots.joint import get_joint_positions, get_custom_limits, get_movable_joints, set_joint_positions, \
    get_configuration, get_custom_max_velocity
from pybullet_planning.interfaces.robots.collision import get_collision_fn
from pybullet_planning.interfaces.robots.body import clone_body, remove_body, get_link_pose
from .ladder_graph import LadderGraph, EdgeBuilder, append_ladder_graph
from .dag_search import DAGSearch

#####################################

NullSpace = namedtuple('Nullspace', ['lower', 'upper', 'range', 'rest'])

def get_null_space(robot, joints, custom_limits={}):
    rest_positions = get_joint_positions(robot, joints)
    lower, upper = get_custom_limits(robot, joints, custom_limits)
    lower = np.maximum(lower, -10*np.ones(len(joints)))
    upper = np.minimum(upper, +10*np.ones(len(joints)))
    joint_ranges = 10*np.ones(len(joints))
    return NullSpace(list(lower), list(upper), list(joint_ranges), list(rest_positions))

def plan_cartesian_motion(robot, first_joint, target_link, waypoint_poses,
                          max_iterations=200, custom_limits={}, get_sub_conf=False, **kwargs):
    """Compute a joint trajectory for a given sequence of workspace poses. Only joint limit is considered.
    Collision checking using `get_collision_fn` is often performed on the path computed by this function.

    Parameters
    ----------
    robot : int
        robot body index
    first_joint : int
        the first joint index in the kinematics chain.
    target_link : int
        end effector link index.
    waypoint_poses : a list of Pose
        a list of end effector workspace poses in the world coord.
    max_iterations : int, optional
        [description], by default 200
    custom_limits : dict, optional
        [description], by default {}
    get_sub_conf : bool, optional
        return sub-kinematics chain configuration if set to True, by default False

    Returns
    -------
    [type]
        [description]

    Example
    -------
    ```
    ik_joints = joints_from_names(robot_uid, ik_joint_names)
    ik_tool_link = link_from_name(robot_uid, tool_link_name)
    cart_conf_vals = plan_cartesian_motion(robot_uid, ik_joints[0], ik_tool_link, world_from_ee_poses)
    ```
    """
    from pybullet_planning.interfaces.env_manager.pose_transformation import all_between
    from pybullet_planning.interfaces.robots.link import get_link_subtree, prune_fixed_joints
    from pybullet_planning.interfaces.kinematics import inverse_kinematics_helper, is_pose_close

    # TODO: fix stationary joints
    # TODO: pass in set of movable joints and take least common ancestor
    # TODO: update with most recent bullet updates
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    # TODO: plan a path without needing to following intermediate waypoints

    lower_limits, upper_limits = get_custom_limits(robot, get_movable_joints(robot), custom_limits)
    # TODO might have problem in a dual-arm setting, where the two arms share the same first joint
    selected_links = get_link_subtree(robot, first_joint) # TODO: child_link_from_joint?
    selected_movable_joints = prune_fixed_joints(robot, selected_links)
    assert(target_link in selected_links)
    selected_target_link = selected_links.index(target_link)
    sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    sub_movable_joints = get_movable_joints(sub_robot)
    #null_space = get_null_space(robot, selected_movable_joints, custom_limits=custom_limits)
    null_space = None

    solutions = []
    for target_pose in waypoint_poses:
        for iteration in range(max_iterations):
            sub_kinematic_conf = inverse_kinematics_helper(sub_robot, selected_target_link, target_pose, null_space=null_space)
            if sub_kinematic_conf is None:
                remove_body(sub_robot)
                return None
            set_joint_positions(sub_robot, sub_movable_joints, sub_kinematic_conf)
            if is_pose_close(get_link_pose(sub_robot, selected_target_link), target_pose, **kwargs):
                set_joint_positions(robot, selected_movable_joints, sub_kinematic_conf)
                kinematic_conf = get_configuration(robot)
                if not all_between(lower_limits, kinematic_conf, upper_limits):
                    #movable_joints = get_movable_joints(robot)
                    #print([(get_joint_name(robot, j), l, v, u) for j, l, v, u in
                    #       zip(movable_joints, lower_limits, kinematic_conf, upper_limits) if not (l <= v <= u)])
                    #print("Limits violated")
                    #wait_for_user()
                    remove_body(sub_robot)
                    return None
                #print("IK iterations:", iteration)
                if not get_sub_conf:
                    solutions.append(kinematic_conf)
                else:
                    solutions.append(sub_kinematic_conf)
                break
        else:
            remove_body(sub_robot)
            return None
    remove_body(sub_robot)
    return solutions

def sub_inverse_kinematics(robot, first_joint, target_link, target_pose, **kwargs):
    solutions = plan_cartesian_motion(robot, first_joint, target_link, [target_pose], **kwargs)
    if solutions:
        return solutions[0]
    return None

#####################################

MAX_SAMPLE_ITER = int(1e4)

def plan_cartesian_motion_lg(robot, joints, waypoint_poses, sample_ik_fn=None, collision_fn=None, sample_ee_fn=None,
    max_sample_ee_iter=MAX_SAMPLE_ITER, jump_threshold=None, enforce_start_conf=True, **kwargs):
    """ladder graph cartesian planning, better leveraging ikfast for sample_ik_fn

    Parameters
    ----------
    robot : [type]
        [description]
    joints : [type]
        [description]
    waypoint_poses : [type]
        [description]
    sample_ik_fn : [type], optional
        [description], by default None
    collision_fn : [type], optional
        [description], by default None
    ee_sample_fn : [type], optional
        please please please remember to put an end to the sampling loop!
    jump_threshold : [type], optional

    Returns
    -------
    [type]
        [description]
    """
    from pybullet_planning.interfaces.env_manager.user_io import wait_for_user
    from pybullet_planning.interfaces.env_manager.savers import WorldSaver

    assert sample_ik_fn is not None, 'Sample fn must be specified!'

    # TODO sanity check samplers
    with WorldSaver():
        ik_sols = [[] for _ in range(len(waypoint_poses))]
        for i, task_pose in enumerate(waypoint_poses):
            candidate_poses = [task_pose]
            if sample_ee_fn is not None:
                # extra dof release, copy to reuse generator
                current_ee_fn = copy(sample_ee_fn)
                cnt = 0
                for p in current_ee_fn(task_pose):
                    if cnt > max_sample_ee_iter:
                        warnings.warn('EE dof release generator is called over {} times, likely that you forget to put an exit in the generator. ' + \
                            'We stop generating here for you.')
                        break
                    candidate_poses.append(p)
                    cnt += 1
            for ee_pose in candidate_poses:
                conf_list = sample_ik_fn(ee_pose)
                if collision_fn is not None:
                    conf_list = [conf for conf in conf_list if conf and not collision_fn(conf, **kwargs)]
                ik_sols[i].extend(conf_list)

    # assemble the ladder graph
    dof = len(joints)
    graph = LadderGraph(dof)
    graph.resize(len(ik_sols))

    # assign rung data
    for pt_id, ik_confs_pt in enumerate(ik_sols):
        graph.assign_rung(pt_id, ik_confs_pt)

    joint_jump_threshold = None
    if jump_threshold:
        joint_jump_threshold = []
        for joint in joints:
            if joint in jump_threshold:
                joint_jump_threshold.append(jump_threshold[joint])

    # build edges within current pose family
    for i in range(graph.get_rungs_size()-1):
        st_rung_id = i
        end_rung_id = i + 1
        jt1_list = graph.get_data(st_rung_id)
        jt2_list = graph.get_data(end_rung_id)
        st_size = graph.get_rung_vert_size(st_rung_id)
        end_size = graph.get_rung_vert_size(end_rung_id)
        # if st_size == 0 or end_size == 0:
        #     print(ik_sols)

        if st_size == 0:
            print('Ladder graph not valid: rung {}/{} is a zero size rung'.format(st_rung_id, graph.get_rungs_size()))
            return None, None
        if end_size == 0:
            print('Ladder graph not valid: rung {}/{} is a zero size rung'.format(end_rung_id, graph.get_rungs_size()))
            return None, None

        # TODO: preference_cost using pose deviation
        # fully-connected ladder graph
        edge_builder = EdgeBuilder(st_size, end_size, dof,
            jump_threshold=joint_jump_threshold,
            preference_cost=1.0)
        for k in range(st_size):
            st_jt_id = k * dof
            for j in range(end_size):
                end_jt_id = j * dof
                edge_builder.consider(jt1_list[st_jt_id : st_jt_id+dof], jt2_list[end_jt_id : end_jt_id+dof], j)
            edge_builder.next(k)

        # print('({}) rung'.format(i))
        # print('max_dtheta_: ', edge_builder.max_dtheta_)
        # print('edge_scratch_: ', edge_builder.edge_scratch_)
        # print('result: ', edge_builder.result)
        # TODO: more report information here
        if not edge_builder.has_edges:
            print('Ladder graph: no edge built between {}-{} | joint threshold: {}, max delta jt: {}'.format(
                st_rung_id, end_rung_id, joint_jump_threshold, edge_builder.max_dtheta_))
            return None, None

        edges = edge_builder.result
        graph.assign_edges(i, edges)

    # * use current conf in the env as start_conf
    if enforce_start_conf:
        start_conf = get_joint_positions(robot, joints)
        st_graph = LadderGraph(graph.dof)
        st_graph.resize(1)
        st_graph.assign_rung(0, [start_conf])
        unified_graph = append_ladder_graph(st_graph, graph, jump_threshold=joint_jump_threshold)
        if unified_graph is None:
            return None, None
    else:
        unified_graph = graph

    # perform DAG search
    dag_search = DAGSearch(unified_graph)
    min_cost = dag_search.run()
    # list of confs
    path = dag_search.shortest_path()
    if enforce_start_conf:
        # delete the start conf
        del path[0]

    assert len(path) == len(waypoint_poses)
    if len(path) == 0 or min_cost == np.inf:
        return None, None
    return path, min_cost
