import os
import pytest
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions
from pybullet_planning import dump_world, set_pose
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

# @pytest.fixture
# def collision_diagnosis():
#     return True
#     # return False

@pytest.mark.wip
def test_collision_fn(viewer, robot_path, ee_path, workspace_path, attach_obj_path, obstacle_obj_path):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
        workspace = load_pybullet(workspace_path, fixed_base=True)
        ee_body = create_obj(ee_path)
        attached_bar_body = create_obj(attach_obj_path)
        box_body = create_obj(obstacle_obj_path)
        assert isinstance(robot, int) and isinstance(ee_body, int)

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

    workspace_robot_disabled_link_names = [('robot_base_link', 'MIT_3412_robot_base_plate'),
        ('robot_link_1', 'MIT_3412_robot_base_plate'), ('robot_link_2', 'MIT_3412_robot_base_plate')]

    bb_disabled_links = get_body_body_disabled_collisions(robot, workspace, workspace_robot_disabled_link_names)
    for bbl in bb_disabled_links:
        assert isinstance(bbl[0], tuple) and isinstance(bbl[1], tuple)
        if bbl[0][0] == robot:
            assert has_link(robot, get_link_name(robot, bbl[0][1]))
            assert bbl[1][0] == workspace and has_link(workspace, get_link_name(workspace, bbl[1][1]))
        else:
            assert bbl[0][0] == workspace and has_link(workspace, get_link_name(workspace, bbl[0][1]))
            assert bbl[1][0] == robot and has_link(robot, get_link_name(robot, bbl[1][1]))

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
    # TODO: robot links self-collision
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links)
    conf = [-1.029744, -1.623156, 2.844887, -0.977384, 1.58825, 0.314159]
    with pytest.warns(UserWarning, match='body link-link collision'):
        assert collision_fn(conf, diagnosis=True)

    # TODO: robot links - holding attachment self-collision
    collision_fn = get_collision_fn(robot, ik_joints, obstacles=[],
                                    attachments=attachments, self_collisions=True,
                                    disabled_collisions=self_collision_links)
    # conf = [0.035, -2.269, 2.339, 1.222, 1.414, 0.314]
    conf = [0.035000000000000003, -2.2690000000000001, 2.4430000000000001, 1.117, 1.6579999999999999, 0.105]
    with pytest.warns(UserWarning, match='body link-link collision'):
        assert collision_fn(conf, diagnosis=True)

    # TODO: robot links to obstacles (w/o links) collision

    # TODO: robot links to multi-link obstacle collision

    # TODO: attachment to obstacles (w/o links) collision

    # * collision checking exoneration
    # TODO: robot links to multi-links obstacles (w/o links) collision exoneration

    # TODO: attachment to obstacles (w/o links) collision exoneration

    # TODO: attachment to multi-links obstacles (w/o links) collision exoneration

    # * joint value overflow checking & exoneration


    if has_gui() : wait_for_user()
