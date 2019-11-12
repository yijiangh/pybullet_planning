import os
import pytest
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name
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

    # * attach the end effector
    ee_link_pose = get_link_pose(robot, tool_attach_link)
    set_pose(ee_body, ee_link_pose)
    ee_attach = create_attachment(robot, tool_attach_link, ee_body)
    assert isinstance(ee_attach, Attachment)
    ee_attach.assign()

    # * attach the bar
    ee_link_from_tcp = Pose(point=(0.094, 0, 0))
    set_pose(attached_bar_body, multiply(ee_link_pose, ee_link_from_tcp))

    if has_gui() : wait_for_user()
    # * collision checks
    # TODO: robot links self-collision

    # TODO: robot links - holding attachment self-collision

    # TODO: robot links to obstacles (w/o links) collision

    # TODO: robot links to multi-link obstacle collision

    # TODO: attachment to obstacles (w/o links) collision

    # * collision checking exoneration
    # TODO: robot links to multi-links obstacles (w/o links) collision exoneration

    # TODO: attachment to obstacles (w/o links) collision exoneration

    # TODO: attachment to multi-links obstacles (w/o links) collision exoneration

