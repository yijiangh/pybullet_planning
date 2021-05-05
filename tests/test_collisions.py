import os
import sys
import pytest
import numpy as np
from termcolor import cprint
from pybullet_planning import BASE_LINK
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name
from pybullet_planning import dump_world, set_pose, unit_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links


@pytest.fixture
def robot_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')

@pytest.fixture
def workspace_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')

@pytest.fixture
def ee_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'dms_bar_gripper.obj')

@pytest.fixture
def attach_obj_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'bar_attachment.obj')

@pytest.fixture
def obstacle_obj_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'box_obstacle.obj')

@pytest.fixture
def convex_collision_objects():
    here = os.path.dirname(__file__)
    test_dir = os.path.join(here, 'test_data')
    link_4_group = os.path.join(test_dir, 'link_4.obj')
    link_4_export = os.path.join(test_dir, 'link_4_export.obj')
    link_4_unified = os.path.join(test_dir, 'link_4_unified.obj')
    link_4_collider = os.path.join(test_dir, 'link_4_collider.obj')
    return link_4_group, link_4_export, link_4_unified, link_4_collider

@pytest.mark.collision_fn
def test_collision_fn(viewer, robot_path, ee_path, workspace_path, attach_obj_path, obstacle_obj_path):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
        workspace = load_pybullet(workspace_path, fixed_base=True)
        ee_body = create_obj(ee_path)
        attached_bar_body = create_obj(attach_obj_path)
        box_body = create_obj(obstacle_obj_path)
        if sys.version_info[0] >= 3:
            assert isinstance(robot, int) and isinstance(ee_body, int)
        else:
            assert isinstance(robot, (long, int)) and isinstance(ee_body, (long, int))
    dump_world()

    # * adjust camera pose (optional)
    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
        set_camera_pose(tuple(camera_pt), camera_base_pt)

    ik_joints = get_movable_joints(robot)
    robot_start_conf = [0,-1.65715,1.71108,-1.62348,0,0]
    set_joint_positions(robot, ik_joints, robot_start_conf)

    tool_attach_link_name = 'ee_link'
    tool_attach_link = link_from_name(robot, tool_attach_link_name)
    assert isinstance(tool_attach_link, int)

    robot_self_collision_disabled_link_names = [('base_link', 'shoulder_link'),
        ('ee_link', 'wrist_1_link'), ('ee_link', 'wrist_2_link'),
        ('ee_link', 'wrist_3_link'), ('forearm_link', 'upper_arm_link'),
        ('forearm_link', 'wrist_1_link'), ('shoulder_link', 'upper_arm_link'),
        ('wrist_1_link', 'wrist_2_link'), ('wrist_1_link', 'wrist_3_link'),
        ('wrist_2_link', 'wrist_3_link')]
    self_collision_links = get_disabled_collisions(robot, robot_self_collision_disabled_link_names)
    assert all(isinstance(lp, tuple) for lp in self_collision_links)
    for lp in self_collision_links:
        assert len(lp) == 2 and has_link(robot, get_link_name(robot, lp[0])) and has_link(robot, get_link_name(robot, lp[1]))

    extra_disabled_link_names = [('base_link', 'MIT_3412_robot_base_plate'),
                                 ('shoulder_link', 'MIT_3412_robot_base_plate')]
    extra_disabled_collisions = get_body_body_disabled_collisions(robot, workspace, extra_disabled_link_names)
    for bbl in list(extra_disabled_collisions):
        assert isinstance(bbl[0], tuple) and isinstance(bbl[1], tuple)
        if bbl[0][0] == robot:
            assert has_link(robot, get_link_name(robot, bbl[0][1]))
            assert bbl[1][0] == workspace and has_link(workspace, get_link_name(workspace, bbl[1][1]))
        else:
            assert bbl[0][0] == workspace and has_link(workspace, get_link_name(workspace, bbl[0][1]))
            assert bbl[1][0] == robot and has_link(robot, get_link_name(robot, bbl[1][1]))

    assert are_links_adjacent(robot, link_from_name(robot, 'wrist_3_link'), tool_attach_link)
    extra_disabled_collisions.add(((robot, link_from_name(robot, 'wrist_3_link')), (ee_body, BASE_LINK)))
    print('extra diasabled: {}'.format(extra_disabled_collisions))

    # * attach the end effector
    ee_link_pose = get_link_pose(robot, tool_attach_link)
    set_pose(ee_body, ee_link_pose)
    ee_attach = create_attachment(robot, tool_attach_link, ee_body)
    assert isinstance(ee_attach, Attachment)
    ee_attach.assign()

    # * attach the bar
    ee_link_from_tcp = Pose(point=(0.094, 0, 0))
    set_pose(attached_bar_body, multiply(ee_link_pose, ee_link_from_tcp))
    bar_attach = create_attachment(robot, tool_attach_link, attached_bar_body)
    assert isinstance(bar_attach, Attachment)
    bar_attach.assign()

    attachments = [ee_attach, bar_attach]

    # * collision checks
    print('#'*10)
    print('robot links self-collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links)
    conf = [-1.029744, -1.623156, 2.844887, -0.977384, 1.58825, 0.314159]
    # with pytest.warns(UserWarning, match='moving body link - moving body link collision'):
    assert collision_fn(conf, diagnosis=has_gui())

    print('#'*10)
    print('robot links - holding attachment self-collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [0.03500, -2.26900, 2.44300, 1.117, 1.6579, 0.105]
    # with pytest.warns(UserWarning, match='moving body link - attachement collision'):
    assert collision_fn(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('robot links to obstacles (w/o links) collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.105, -0.76800000000000002, 1.292, -0.61099999999999999, 1.484, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('robot links to multi-link obstacle collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                    attachments=[], self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.17499999999999999, -3.194, 0.33200000000000002, -1.6579999999999999, 1.431, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('attachment to obstacles (w/o links) collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace, box_body],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-2.8100000000000001, -1.484, -1.9199999999999999, -1.6579999999999999, 1.431, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('attachment to multi-link obstacle collision')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    conf = [-0.17499999999999999, -2.4780000000000002, 0.33200000000000002, -1.6579999999999999, 1.431, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    print('\n')

    # * collision checking exoneration
    print('#'*10)
    print('self-link collision disable')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=[], self_collisions=False)
    conf = [-1.029744, -1.623156, 2.844887, -0.977384, 1.58825, 0.314159]
    assert not collision_fn(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('robot links to obstacle collision exoneration')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            )
    collision_fn_disable = get_collision_fn(robot, ik_joints, obstacles=[box_body],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            extra_disabled_collisions=extra_disabled_collisions.union(
                                                [((robot, link_from_name(robot, 'forearm_link')),
                                                  (box_body, BASE_LINK))]),
                                            )
    conf = [-3.2639999999999998, -2.6880000000000002, -0.85499999999999998, -1.536, 3.0369999999999999, -0.070000000000000007]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    assert not collision_fn_disable(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('robot links to multi-links obstacles collision exoneration')
    set_pose(workspace, Pose(point=(0,0,0.03)))
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            )
    collision_fn_disable = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=self_collision_links,
                                            extra_disabled_collisions=extra_disabled_collisions.union(
                                                [((robot, link_from_name(robot, 'upper_arm_link')),
                                                  (workspace, link_from_name(workspace, 'MIT_3412_robot_base_plate')))]),
                                            )
    conf = [-3.0019999999999998, -1.8680000000000001, 0.33200000000000002, -1.6579999999999999, 1.431, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    assert not collision_fn_disable(conf, diagnosis=has_gui())
    set_pose(workspace, Pose(point=(0,0,0)))
    print('\n')

    print('#'*10)
    print('attachment to obstacles collision exoneration')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace, box_body],
                                    attachments=[ee_attach], self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    collision_fn_disabled = get_collision_fn(robot, ik_joints, obstacles=[workspace, box_body],
                                             attachments=[ee_attach], self_collisions=True,
                                             disabled_collisions=self_collision_links,
                                             extra_disabled_collisions=extra_disabled_collisions.union(
                                                        [((ee_attach.child, BASE_LINK), (box_body, BASE_LINK))]),
                                            )
    conf = [-3.0369999999999999, -1.6060000000000001, -1.99, -0.92500000000000004, 1.78, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    assert not collision_fn_disable(conf, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('attachment to multi-links obstacles collision exoneration')
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                    attachments=[ee_attach], self_collisions=True,
                                    disabled_collisions=self_collision_links,
                                    extra_disabled_collisions=extra_disabled_collisions)
    collision_fn_disabled = get_collision_fn(robot, ik_joints, obstacles=[workspace],
                                             attachments=[ee_attach], self_collisions=True,
                                             disabled_collisions=self_collision_links,
                                             extra_disabled_collisions=extra_disabled_collisions.union(
                                                        [((workspace, link_from_name(workspace, 'MIT_3412_fab_table')),
                                                          (ee_attach.child, BASE_LINK))]),
                                            )
    conf = [-2.8450000000000002, -2.1469999999999998, -1.99, -0.92500000000000004, 1.78, 0.105]
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert collision_fn(conf, diagnosis=has_gui())
    assert not collision_fn_disable(conf, diagnosis=has_gui())
    print('\n')

    # * joint value overflow checking & exoneration
    print('joint value overflow checking & exoneration')
    def get_custom_limits_from_name(robot, joint_limits):
        return {joint_from_name(robot, joint): limits
                for joint, limits in joint_limits.items()}
    custom_limits = get_custom_limits_from_name(robot, {'shoulder_pan_joint':(-7.9, 0), 'elbow_joint':(-8.0, 0)})
    collision_fn = get_collision_fn(robot, ik_joints)
    collision_fn_disable = get_collision_fn(robot, ik_joints, custom_limits=custom_limits)
    conf = [-7.8450000000000002, -2.1469999999999998, -7.99, -0.92500000000000004, 1.78, 0.105]
    # with pytest.warns(UserWarning, match='joint limit violation!'):
    assert collision_fn(conf, diagnosis=has_gui())
    assert not collision_fn_disable(conf, diagnosis=has_gui())
    print('\n')

@pytest.mark.floatting_collision_fn
def test_floating_collsion_fn(viewer, robot_path, ee_path, workspace_path, attach_obj_path, obstacle_obj_path):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
        workspace = load_pybullet(workspace_path, fixed_base=True)
        ee_body = create_obj(ee_path)
        attached_bar_body = create_obj(attach_obj_path)
        box_body = create_obj(obstacle_obj_path)
    dump_world()

    # * adjust camera pose (optional)
    if has_gui():
        camera_base_pt = (0,0,0)
        camera_pt = np.array(camera_base_pt) + np.array([1, -0.5, 0.5])
        set_camera_pose(tuple(camera_pt), camera_base_pt)

    ik_joints = get_movable_joints(robot)
    robot_start_conf = [0,-1.65715,1.71108,-1.62348,0,0]
    set_joint_positions(robot, ik_joints, robot_start_conf)

    tool_attach_link_name = 'ee_link'
    tool_attach_link = link_from_name(robot, tool_attach_link_name)
    assert isinstance(tool_attach_link, int)

    print('#'*10)
    print('floating body to obstacles collision exoneration')
    conf = [-3.0369999999999999, -1.6060000000000001, -1.99, -0.92500000000000004, 1.78, 0.105]
    set_joint_positions(robot, ik_joints, conf)
    world_from_tool0 = get_link_pose(robot, tool_attach_link)
    fb_collision_fn = get_floating_body_collision_fn(ee_body, obstacles=[box_body],
                                                     attachments=[], disabled_collisions=[])
    fb_collision_fn_disable = get_floating_body_collision_fn(ee_body, obstacles=[box_body],
                                                     attachments=[], disabled_collisions=
                                                     {((box_body, BASE_LINK), (ee_body, BASE_LINK))})
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert fb_collision_fn(world_from_tool0, diagnosis=has_gui())
    assert not fb_collision_fn_disable(world_from_tool0, diagnosis=has_gui())
    print('\n')

    print('#'*10)
    print('attachment to multi-links obstacles collision exoneration')
    conf = [-2.8450000000000002, -2.1469999999999998, -1.99, -0.92500000000000004, 1.78, 0.105]
    set_joint_positions(robot, ik_joints, conf)
    world_from_tool0 = get_link_pose(robot, tool_attach_link)
    fb_collision_fn = get_floating_body_collision_fn(ee_body, obstacles=[workspace],
                                                     attachments=[], disabled_collisions=[])
    fb_collision_fn_disable = get_floating_body_collision_fn(ee_body, obstacles=[workspace],
                                                     attachments=[], disabled_collisions=
                                                     {((workspace, link_from_name(workspace, 'MIT_3412_fab_table')),
                                                       (ee_body, BASE_LINK))})
    # with pytest.warns(UserWarning, match='moving body - body collision!'):
    assert fb_collision_fn(world_from_tool0, diagnosis=has_gui())
    assert not fb_collision_fn_disable(world_from_tool0, diagnosis=has_gui())
    print('\n')


@pytest.mark.convex_collision
def test_convex_collision(convex_collision_objects, viewer):
    link_4_group, link_4_export, link_4_unified, link_4_collider = convex_collision_objects
    connect(use_gui=viewer)
    with HideOutput():
        group_body = create_obj(link_4_group)
        export_body = create_obj(link_4_export)
        unified_body = create_obj(link_4_unified)
        collider_body = create_obj(link_4_collider)
    # dump_world()

    # * VHACD exported
    cprint('VHACD exported')
    fb_collision_fn = get_floating_body_collision_fn(collider_body, obstacles=[group_body],
                                                     attachments=[], disabled_collisions=[])
    assert not fb_collision_fn(unit_pose(), diagnosis=has_gui())
    cprint('Decomposed, convexified objects exported as individual objects in one file not in collision.', 'green')

    # * Rhino exported, wrong settings
    cprint('rhino exported, wrong setting')
    fb_collision_fn = get_floating_body_collision_fn(collider_body, obstacles=[export_body],
                                                     attachments=[], disabled_collisions=[])
    assert fb_collision_fn(unit_pose(), diagnosis=has_gui())
    cprint('Convexified object concatenated in one file in collision. (without`o xxx` as separator)', 'red')

    cprint('unified mesh')
    fb_collision_fn = get_floating_body_collision_fn(collider_body, obstacles=[unified_body],
                                                     attachments=[], disabled_collisions=[])
    assert fb_collision_fn(unit_pose(), diagnosis=has_gui())
    cprint('Concave object will be auto-convexified as one object in collision.', 'red')

