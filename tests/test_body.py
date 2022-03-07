import os
import sys
import random
import pytest
import pybullet as pb
import numpy as np
import pybullet_planning as pp
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, wait_if_gui, apply_alpha
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints
from pybullet_planning import dump_world, set_pose, set_color
from pybullet_planning import clone_body, create_box, read_obj
from pybullet_planning import Pose
from termcolor import cprint
from pybullet_planning import LOGGER

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

@pytest.mark.create_body
@pytest.mark.parametrize("file_format",[
    'obj',
    'stl',
    # 'dae',
    # 'ply',
    ])
def test_create_body(viewer, file_format):
    here = os.path.dirname(__file__)
    if file_format == 'dae':
        path = os.path.join(here, 'test_data', 'kuka_kr6_r900/meshes/kr6_agilus/visual/base_link.dae', )
    else:
        path = os.path.join(here, 'test_data', 'link_4.' + file_format)
    connect(use_gui=viewer)
    try:
        with HideOutput():
            body = create_obj(path)
        wait_if_gui('Body created.')
    finally:
        disconnect()


@pytest.mark.read_obj
def test_read_obj(viewer):
    here = os.path.dirname(__file__)
    path = os.path.join(here, 'test_data', 'box_obstacle.obj')
    path2 = os.path.join(here, 'test_data', 'link_4.obj')
    connect(use_gui=viewer)
    try:
        # obj without `o group_name`
        mesh = pp.read_obj(path, decompose=False)
        assert(len(mesh.vertices)==48)
        assert(len(mesh.faces)==6)
        meshes = pp.read_obj(path, decompose=True)
        assert(len(meshes[None].vertices)==48)
        assert(len(meshes[None].faces)==6)

        # obj with multiple `o group_name`
        meshes2 = pp.read_obj(path2, decompose=True)
        assert(len(meshes2)==5)
        assert(len(meshes2['convex_0'].vertices)==60)
        assert(len(meshes2['convex_1'].faces)==124)
        assert(len(meshes2['convex_4'].vertices)==27)
    finally:
        disconnect()

@pytest.mark.geom_data_index
def test_geom_data_index(viewer):
    verbose = True
    here = os.path.dirname(__file__)
    c1_path = os.path.join(here, 'test_data', 'c1', 'urdf', 'c1.urdf')
    kuka_path = os.path.join(here, 'test_data', 'kuka_kr6_r900', 'urdf', 'kuka_kr6_r900.urdf')
    mesh_path = os.path.join(here, 'test_data', 'duck.obj')

    connect(use_gui=viewer)
    with HideOutput():
        # just to shift body indices
        for _ in range(random.choice(range(10))):
            pp.create_box(0.1,0.1,0.1)

        bodies = []
        geom_types = []
        body_dimensions = []

        dimensions = [1.2,2.1,13.1]
        bodies.append(pp.create_box(*dimensions, color=apply_alpha(pp.RED, 0.2)))
        geom_types.append(pb.GEOM_BOX)
        body_dimensions.append(dimensions)

        # radius, height
        dimensions = [0.6,3]
        bodies.append(pp.create_cylinder(*dimensions, color=apply_alpha(pp.GREEN, 0.2)))
        geom_types.append(pb.GEOM_CYLINDER)
        # pb collision/visual data returns [height, radius, 0]
        body_dimensions.append(dimensions[::-1] + [0])

        # radius, height
        dimensions = [0.7,4]
        bodies.append(pp.create_capsule(*dimensions, color=apply_alpha(pp.BLUE, 0.2)))
        geom_types.append(pb.GEOM_CAPSULE)
        # pb collision/visual data returns [height, radius, 0]
        body_dimensions.append(dimensions[::-1] + [0])

        # radius
        dimensions = [0.8]
        bodies.append(pp.create_sphere(*dimensions, color=apply_alpha(pp.TAN, 0.2)))
        geom_types.append(pb.GEOM_SPHERE)
        body_dimensions.append(dimensions*3)

        dimensions = [0.01]
        bodies.append(pp.create_obj(mesh_path, color=apply_alpha(pp.TAN, 0.2), scale=dimensions[0]))
        geom_types.append(pb.GEOM_MESH)
        body_dimensions.append(dimensions*3)

        robot_scale = 0.01
        robot = load_pybullet(kuka_path, fixed_base=True, scale=robot_scale)
        c1_scale = 0.001
        c1 = load_pybullet(c1_path, fixed_base=True, scale=c1_scale)

    # * robot
    cprint('kuka robot', 'cyan')
    for link in pp.get_all_links(robot):
        link_name = get_link_name(robot, link)
        print('====')
        cprint(link_name, 'cyan')
        cdatas = pp.get_collision_data(robot, link)
        vdatas = pp.get_visual_data(robot, link)
        if 'robot_link' in link_name or 'robot_base_link' == link_name:
            assert len(cdatas) == 1 and len(vdatas) == 1
        else:
            assert len(cdatas) == 0 and len(vdatas) == 0
        for cdata in cdatas:
            if verbose: print(cdata)
            assert(cdata.objectUniqueId == robot)
            assert(cdata.linkIndex == link)
            assert(cdata.geometry_type == pb.GEOM_MESH)
            assert(cdata.dimensions == tuple([robot_scale]*3))
        for vdata in vdatas:
            if verbose: print(vdata)
            assert(vdata.objectUniqueId == robot)
            assert(vdata.linkIndex == link)
            assert(vdata.visualGeometryType == pb.GEOM_MESH)
            assert(vdata.dimensions == tuple([robot_scale]*3))

    # * c1
    cprint('C1', 'cyan')
    num_meshes_from_link_name = {
        'gripper_base' : 16,
        'gripper_jaw' : 3,
        'clamp_jaw' : 3,
    }
    for link in pp.get_all_links(c1):
        link_name = get_link_name(c1, link)
        print('====')
        cprint(link_name, 'cyan')
        cdatas = pp.get_collision_data(c1, link)
        vdatas = pp.get_visual_data(c1, link)
        assert len(cdatas) == num_meshes_from_link_name[link_name]
        assert len(vdatas) == num_meshes_from_link_name[link_name]
        for cdata in cdatas:
            if verbose: print(cdata)
            # ! in this case the objectUniqueId and linkIndex can be random...
            # assert(cdata.objectUniqueId == c1)
            # assert(cdata.linkIndex == link)
            assert(cdata.geometry_type == pb.GEOM_MESH)
            assert(cdata.dimensions == tuple([1e-3*c1_scale]*3))
        for vdata in vdatas:
            if verbose: print(vdata)
            assert(vdata.objectUniqueId == c1)
            assert(vdata.linkIndex == link)
            assert(vdata.visualGeometryType == pb.GEOM_MESH)
            assert(vdata.dimensions == tuple([1e-3*c1_scale]*3))

    # primitive geometries
    for body, geom_type, b_dims in zip(bodies, geom_types, body_dimensions):
        cprint(pp.SHAPE_TYPES[geom_type], 'cyan')
        cdatas = pp.get_collision_data(body)
        assert len(cdatas) == 1
        cdata = cdatas[0]
        if verbose: print(cdata)
        # print(pb.getCollisionShapeData(body, pp.BASE_LINK, physicsClientId=pp.CLIENT))
        assert(cdata.objectUniqueId == body)
        assert(cdata.linkIndex == pp.BASE_LINK)
        assert(cdata.geometry_type == geom_type)
        if geom_type == pb.GEOM_MESH:
            # ! it seems that pybullet does v-hacd inside and the dimensions will always be (1,1,1)
            assert(cdata.dimensions == (1.0, 1.0, 1.0))
        else:
            assert(cdata.dimensions == tuple(b_dims))

        vdatas = pp.get_visual_data(body)
        assert len(vdatas) == 1
        vdata = vdatas[0]
        if verbose: print(vdata)
        assert(vdata.objectUniqueId == body)
        assert(vdata.linkIndex == pp.BASE_LINK)
        assert(vdata.visualGeometryType == geom_type)
        if geom_type == pb.GEOM_SPHERE:
            assert(vdata.dimensions == (b_dims[0], 0.0, 0.0))
        else:
            assert(vdata.dimensions == tuple(b_dims))

