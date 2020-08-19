import os
import warnings
from itertools import tee
from copy import copy
from collections import namedtuple
import numpy as np
import pybullet as p

from pybullet_planning.utils import MAX_DISTANCE
from pybullet_planning.interfaces.robots.joint import get_joint_positions, get_custom_limits, get_movable_joints, set_joint_positions, \
    get_configuration
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
                          max_iterations=200, custom_limits={}, **kwargs):
    """[summary]

    Parameters
    ----------
    robot : [type]
        [description]
    first_joint : [type]
        [description]
    target_link : [type]
        [description]
    waypoint_poses : [type]
        [description]
    max_iterations : int, optional
        [description], by default 200
    custom_limits : dict, optional
        [description], by default {}

    Returns
    -------
    [type]
        [description]
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
                solutions.append(kinematic_conf)
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
    max_sample_ee_iter=MAX_SAMPLE_ITER, **kwargs):
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

    Returns
    -------
    [type]
        [description]
    """

    assert sample_ik_fn is not None, 'Sample fn must be specified!'
    # TODO sanity check samplers
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
            if collision_fn is None:
                conf_list = [conf for conf in conf_list if conf and not collision_fn(conf, **kwargs)]
            ik_sols[i].extend(conf_list)

    # assemble the ladder graph
    dof = len(joints)
    graph = LadderGraph(dof)
    graph.resize(len(ik_sols))

    # assign rung data
    for pt_id, ik_confs_pt in enumerate(ik_sols):
        graph.assign_rung(pt_id, ik_confs_pt)

    # build edges within current pose family
    for i in range(graph.get_rungs_size()-1):
        st_id = i
        end_id = i + 1
        jt1_list = graph.get_data(st_id)
        jt2_list = graph.get_data(end_id)
        st_size = graph.get_rung_vert_size(st_id)
        end_size = graph.get_rung_vert_size(end_id)
        # if st_size == 0 or end_size == 0:
        #     print(ik_sols)

        assert st_size > 0, 'Ladder graph not valid: rung {}/{} is a zero size rung'.format(st_id, graph.get_rungs_size())
        assert end_size > 0, 'Ladder graph not valid: rung {}/{} is a zero size rung'.format(end_id, graph.get_rungs_size())

        # TODO: preference_cost
        # fully-connected ladder graph
        edge_builder = EdgeBuilder(st_size, end_size, dof, preference_cost=1.0)
        for k in range(st_size):
            st_id = k * dof
            for j in range(end_size):
                end_id = j * dof
                edge_builder.consider(jt1_list[st_id : st_id+dof], jt2_list[end_id : end_id+dof], j)
            edge_builder.next(k)
        edges = edge_builder.result
        # if not edge_builder.has_edges and verbose:
        #     # TODO: more report information here
        #     print('no edges!')
        graph.assign_edges(i, edges)

    # * use current conf in the env as start_conf
    start_conf = get_joint_positions(robot, joints)
    st_graph = LadderGraph(graph.dof)
    st_graph.resize(1)
    st_graph.assign_rung(0, [start_conf])
    unified_graph = append_ladder_graph(st_graph, graph)

    # perform DAG search
    dag_search = DAGSearch(unified_graph)
    min_cost = dag_search.run()
    # list of confs
    path = dag_search.shortest_path()
    # delete the start conf
    del path[0]

    assert len(path) == len(waypoint_poses)
    if len(path) == 0:
        return None
    return path, min_cost
