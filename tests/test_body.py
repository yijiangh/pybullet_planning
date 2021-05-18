import os
import sys
import pytest
import pybullet
import warnings
import pybullet_planning as pp
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, wait_if_gui
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints
from pybullet_planning import dump_world, set_pose
from pybullet_planning import clone_body, create_box
from pybullet_planning import Pose


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

def test_create_ee_link(viewer, robot_path, ee_path):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
        ee_body = create_obj(ee_path)
        if sys.version_info[0] >= 3:
            assert isinstance(robot, int) and isinstance(ee_body, int)
        else:
            assert isinstance(robot, (long, int)) and isinstance(ee_body, (long, int))
    dump_world()

    tool_attach_link_name = 'ee_link'
    tool_attach_link = link_from_name(robot, tool_attach_link_name)
    assert isinstance(tool_attach_link, int)

    # * need to set end effector to robot's tool link pose before doing "in-place" attaching
    ee_link_pose = get_link_pose(robot, tool_attach_link)
    set_pose(ee_body, ee_link_pose)
    ee_attach = create_attachment(robot, tool_attach_link, ee_body)
    assert isinstance(ee_attach, Attachment)
    ee_attach.assign()

    if has_gui() : wait_for_user()

def test_moving_links_joints(viewer, robot_path, workspace_path):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
        workspace = load_pybullet(workspace_path, fixed_base=True)
        assert isinstance(robot, int) and isinstance(workspace, int)
    dump_world()

    # * in pybullet, each body have an unique integer index
    # TODO: migrate the notes below to docs
    # within a multibody, bodies in links do not have a body index, but have a link index instead.
    # the link body indices are generated according to their parent joint indices, the shared BASE_LINK get index -1
    # thus, when refering to links within a body, we should always use a (body, [link indices]) tuple.
    movable_joints = get_movable_joints(robot)
    assert isinstance(movable_joints, list) and all([isinstance(mj, int) for mj in movable_joints])
    assert 6 == len(movable_joints)
    assert [b'shoulder_pan_joint', b'shoulder_lift_joint', b'elbow_joint', b'wrist_1_joint', b'wrist_2_joint', b'wrist_3_joint'] == \
        get_joint_names(robot, movable_joints)

    moving_links = get_moving_links(robot, movable_joints)
    assert isinstance(moving_links, list) and all([isinstance(ml, int) for ml in moving_links])
    assert 8 == len(moving_links)
    link_names = [get_link_name(robot, link) for link in moving_links]
    assert ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link', 'tool0'] == \
        link_names

    ws_movable_joints = get_movable_joints(workspace)
    assert 0 == len(ws_movable_joints)

    ws_moving_links = get_moving_links(workspace, ws_movable_joints)
    assert 0 == len(ws_moving_links)

    if has_gui() : wait_for_user()

@pytest.mark.clone
def test_clone_body(viewer, workspace_path, ee_path):
    connect(use_gui=viewer)
    with HideOutput():
        workspace = load_pybullet(workspace_path, fixed_base=True)
        ee_body = create_obj(ee_path)
        box = create_box(0.5, 0.5, 0.5)
        set_pose(box, Pose(point=[1,0,0]))
    move_pose = Pose(point=[0, 3, 0])
    box_pose = Pose(point=[1, 3, 0])

    print('*'*10)
    print('clone workspace')
    c_ws = clone_body(workspace, visual=False)
    set_pose(c_ws, move_pose)

    print('*'*10)
    print('clone visual box')
    c_box_v = clone_body(box, visual=True, collision=False)
    set_pose(c_box_v, box_pose)

    print('*'*10)
    print('clone collision box')
    c_box_c = clone_body(box, visual=False, collision=True)
    set_pose(c_box_c, box_pose)

    print('*'*10)
    print('clone visual ee from obj')
    c_ee_v = clone_body(ee_body, visual=True, collision=False)
    set_pose(c_ee_v, move_pose)

    print('clone collision ee from obj')
    with pytest.raises(pybullet.error):
        warnings.warn('Currently, we do not support clone bodies that are created from an obj file.')
        c_ee_c = clone_body(ee_body, visual=False, collision=True)
        set_pose(c_ee_c, move_pose)

    wait_if_gui()

@pytest.mark.create_body
@pytest.mark.parametrize("file_format",[
    ('obj'),
    ('stl'),
    # ('dae'),
    # ('ply'),
    ]
)
def test_create_body(viewer, file_format):
    here = os.path.dirname(__file__)
    path = os.path.join(here, 'test_data', 'link_4.' + file_format)
    connect(use_gui=viewer)
    try:
        with HideOutput():
            body = create_obj(path)
        wait_if_gui('Body created.')
    finally:
        disconnect()


@pytest.mark.read_obj
def test_create_body(viewer):
    here = os.path.dirname(__file__)
    path = os.path.join(here, 'test_data', 'box_obstacle.obj')
    connect(use_gui=viewer)
    try:
        mesh = pp.read_obj(path, decompose=False)
        assert(len(mesh.vertices)==48)
        assert(len(mesh.faces)==6)
        meshes = pp.read_obj(path, decompose=True)
        assert(len(meshes[None].vertices)==48)
        assert(len(meshes[None].faces)==6)
    finally:
        disconnect()
