import pytest
import os
import numpy as np
import random
from termcolor import cprint

from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui
from pybullet_planning import Pose, Point, Euler, unit_point
from pybullet_planning import multiply, invert
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion, plan_cartesian_motion
from pybullet_planning import inverse_kinematics, sample_tool_ik, interval_generator
from pybullet_planning import dump_world, set_pose, draw_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import plan_cartesian_motion_lg

# ikfast module, compiled by ikfast_pybind
import ikfast_kuka_kr6_r900

@pytest.fixture
def robot_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'kuka_kr6_r900', 'urdf', 'kuka_kr6_r900.urdf')

@pytest.fixture
def link_info():
    ee_link_name = 'robot_tool0'
    robot_base_link_name = 'robot_base_link'
    return ee_link_name, robot_base_link_name

@pytest.mark.cart_plan
@pytest.mark.skip("not fully developed.")
def test_cartesian(viewer, robot_path, link_info):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)

    # * adjust camera pose (optional)
    # has_gui checks if the GUI mode is enabled
    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 1])
        set_camera_pose(tuple(camera_pt), camera_base_pt)

    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)

    ee_link_name, robot_base_link_name = link_info

    # set the robot to a "comfortable" start configuration, optional
    robot_start_conf = [0,-np.pi/2,np.pi/2,0,0,0]
    set_joint_positions(robot, ik_joints, robot_start_conf)

    tool_link = link_from_name(robot, ee_link_name)
    robot_base_link = link_from_name(robot, robot_base_link_name)

    # * draw EE pose
    if has_gui() :
        tcp_pose = get_link_pose(robot, tool_link)
        draw_pose(tcp_pose)

    # total num of path pts
    n_pt = 20
    circle_center = np.array([0.6, 0, 0.2])
    circle_r = 0.1
    # * generate a circle
    ee_poses = []
    full_angle = 2*np.pi
    # full_angle = np.pi
    for a in np.linspace(0.0, full_angle, num=n_pt):
        pt = circle_center + circle_r*np.array([np.cos(a), np.sin(a), 0])
        circ_pose = multiply(Pose(point=pt, euler=Euler(yaw=a+np.pi/2)), Pose(euler=Euler(roll=np.pi*3/4)))
        draw_pose(circ_pose, length=0.05)
        ee_poses.append(circ_pose)

    # * baseline, keeping the EE z axis rotational dof fixed
    path = plan_cartesian_motion(robot, robot_base_link, tool_link, ee_poses)
    if path is None:
        cprint('Gradient-based ik cartesian planning cannot find a plan!', 'red')
    else:
        cprint('Gradient-based ik cartesian planning find a plan!', 'green')
        time_step = 0.03
        for conf in path:
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)

    # * Now, let's try if using ladder graph without releasing the ee dof can give us a good trajectory
    # First, we will need ikfast to obtain ik solution variance, same in Descartes
    ik_fn = ikfast_kuka_kr6_r900.get_ik

    # we have to specify ik fn wrapper and feed it into pychoreo
    def get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints, tool_from_root=None):
        def sample_ik_fn(world_from_tcp):
            if tool_from_root:
                world_from_tcp = multiply(world_from_tcp, tool_from_root)
            return sample_tool_ik(ik_fn, robot, ik_joints, world_from_tcp, robot_base_link, get_all=True)
        return sample_ik_fn
    # ik generation function stays the same for all cartesian processes
    sample_ik_fn = get_sample_ik_fn(robot, ik_fn, robot_base_link, ik_joints)

    # we ignore self collision in this tutorial, the collision_fn only considers joint limit now
    # See : https://github.com/yijiangh/pybullet_planning/blob/dev/tests/test_collisions.py
    # for more info on examples on using collision function
    collision_fn = get_collision_fn(robot, ik_joints, [],
                                       attachments=[], self_collisions=False,
                                    #    disabled_collisions=disabled_collisions,
                                    #    extra_disabled_collisions=extra_disabled_collisions,
                                       custom_limits={})

    # Let's check if our ik sampler is working properly
    for p in ee_poses:
        print('-'*5)
        pb_q = inverse_kinematics(robot, tool_link, p)
        if pb_q is None:
            cprint('pb ik can\'t find an ik solution', 'red')
        qs = sample_ik_fn(p)
        if qs is not None:
            cprint('But Ikfast does find one! {}'.format(qs[0]), 'green')
            # set_joint_positions(robot, ik_joints, qs[0])
            # wait_if_gui()
        else:
            cprint('ikfast can\'t find an ik solution', 'red')

    # * Ok, now we have the ik sampler and collision function ready, let's see if we can find a valid cartesian trajectory!
    path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn)
    if path is None:
        cprint('ladder graph (w/o releasing dof) cartesian planning cannot find a plan!', 'red')
    else:
        cprint('ladder graph (w/o releasing dof) cartesian planning find a plan!', 'green')
        cprint('Cost: {}'.format(cost), 'yellow')
        time_step = 0.03
        for conf in path:
            cprint('conf: {}'.format(conf))
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)

    # * Now, let's see if releasing EE z axis dof can bring down the cost
    # First, let's build an end effector pose sampler to release the EE z axis dof!

    def get_ee_sample_fn(yaw_gen):
        def ee_sample_fn(ee_pose):
            # a finite generator
            for yaw in yaw_gen:
                yield multiply(ee_pose, Pose(euler=Euler(yaw=yaw)))
        return ee_sample_fn

    # by increasing this number we can see the cost go down
    yaw_sample_size = 10
    yaw_gen = np.linspace(0.0, 2*np.pi, num=yaw_sample_size)
    ee_sample_fn = get_ee_sample_fn(yaw_gen)

    path, cost = plan_cartesian_motion_lg(robot, ik_joints, ee_poses, sample_ik_fn, collision_fn, sample_ee_fn=ee_sample_fn)

    if path is None:
        cprint('ladder graph (releasing EE z dof) cartesian planning cannot find a plan!', 'red')
        assert path is not None
    else:
        # the ladder graph cost is just summation of all adjacent joint difference
        # so the following assertion should be true
        conf_array = np.array(path)
        conf_diff = np.abs(conf_array[:-1,:] - conf_array[1:,:])
        np_cost = np.sum(conf_diff)
        assert np.allclose(np_cost, cost), '{} - {}'.format(np_cost, cost)

        cprint('ladder graph (releasing EE z dof) cartesian planning find a plan!', 'cyan')
        cprint('Cost: {}'.format(cost), 'yellow')
        time_step = 0.03
        for conf in path:
            cprint('conf: {}'.format(conf))
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)

    wait_if_gui('Press enter to exit')
