import os
import sys
import random
import pytest
import pybullet as pb
import numpy as np
import warnings
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


@pytest.fixture
def robot_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')

@pytest.fixture
def workspace_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')

@pytest.fixture
def clamp_urdf_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'c1', 'urdf', 'c1.urdf')

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

def inspect_data(body, original_aabb=None, collision_empty=False, visual_empty=False):
    links = pp.get_all_links(body)
    for i, link in enumerate(links):
        collision_data = pp.get_collision_data(body, link)
        visual_data = pp.get_visual_data(body, link)
        LOGGER.debug(f'Link {i}')
        LOGGER.debug('collsion: {}'.format(collision_data))
        LOGGER.debug('visual: {}'.format(visual_data))
        if collision_empty:
            assert len(collision_data) == 0
        if visual_empty:
            assert len(visual_data) == 0
    wait_if_gui()
    if original_aabb:
        orig_extent = pp.get_aabb_extent(original_aabb)
        new_extent = pp.get_aabb_extent(pp.get_aabb(body))
        pp.draw_aabb(original_aabb)
        pp.draw_aabb(pp.get_aabb(body))
        # ! pybullet's AABB is a bit noisy sometimes, with a safety margin added
        assert np.allclose(new_extent, orig_extent, atol=np.max(orig_extent)/10)

@pytest.mark.clone_body_single_obj
@pytest.mark.parametrize("body_name",[
    'obj',
    'box',
    ])
def test_clone_body_single_obj(viewer, body_name, ee_path):
    connect(use_gui=viewer)
    with HideOutput():
        if body_name == 'obj':
            body = create_obj(ee_path, scale=5.0)
        elif body_name == 'box':
            body = create_box(0.2, 0.2, 0.2)
        set_color(body, pp.GREY)

    LOGGER.info('original body')
    original_aabb = pp.get_aabb(body)
    inspect_data(body, original_aabb)

    spacing = 0.5
    cloned_body = clone_body(body)
    set_color(cloned_body, pp.GREEN)
    set_pose(cloned_body, Pose(point=[0, 0, spacing]))
    LOGGER.info('both cloned body')
    inspect_data(cloned_body, original_aabb)

    collision_body = clone_body(body, visual=False)
    set_color(collision_body, pp.RED)
    set_pose(collision_body, Pose(point=[0, 1*spacing, 0]))
    LOGGER.info('collision cloned body')
    inspect_data(collision_body, original_aabb)

    visual_body = clone_body(body, collision=False)
    set_pose(visual_body, Pose(point=[0, 2*spacing, 0]))
    set_color(visual_body, pp.BLUE)
    LOGGER.info('visual cloned body')
    inspect_data(visual_body)

    double_collision_body = clone_body(collision_body)
    set_pose(double_collision_body, Pose(point=[1*spacing, 1*spacing, 0]))
    set_color(double_collision_body, pp.YELLOW)
    LOGGER.info('double collision cloned body')
    inspect_data(double_collision_body, original_aabb)

    double_visual_body = clone_body(visual_body)
    set_pose(double_visual_body, Pose(point=[1*spacing, 2*spacing, 0]))
    set_color(double_visual_body, pp.BROWN)
    LOGGER.info('double visual cloned body')
    inspect_data(double_visual_body)

    pp.disconnect()

@pytest.mark.clone_body_urdf
@pytest.mark.parametrize("body_name",[
    'workspace',
    'robot',
    # 'urdf_link_multi_mesh',
    ])
def test_clone_body_urdf(viewer, body_name, robot_path, clamp_urdf_path, workspace_path):
    connect(use_gui=viewer)
    with HideOutput():
        if body_name == 'workspace':
            body = load_pybullet(workspace_path, fixed_base=True, scale=0.5)
        elif body_name == 'robot':
            body = load_pybullet(robot_path, fixed_base=True)
        elif body_name == 'urdf_link_multi_mesh':
            body = load_pybullet(clamp_urdf_path, fixed_base=True)
        set_color(body, pp.GREY)

    # ! PyBullet does not return any VisualShapeData at all
    LOGGER.info('original body')
    original_aabb = pp.get_aabb(body)
    inspect_data(body, original_aabb, visual_empty=True)

    spacing = 1.5
    # CollisionShapeData's mesh filename will be turned into `unknown file`
    # with VisualShapeData pointing to collision mesh filename.
    cloned_body = clone_body(body)
    set_color(cloned_body, pp.GREEN)
    set_pose(cloned_body, Pose(point=[0, 0, spacing]))
    LOGGER.info('both cloned body')
    inspect_data(cloned_body, original_aabb)

    # ! both collision and visual data will be cloned, behavior same as with above,
    # when `collision=True, visual=True`
    collision_body = clone_body(body, visual=False)
    set_color(collision_body, pp.RED)
    set_pose(collision_body, Pose(point=[0, 1*spacing, 0]))
    LOGGER.info('collision cloned body')
    inspect_data(collision_body, original_aabb)

    # ! both collison and visual data are empty here
    visual_body = clone_body(body, collision=False)
    set_pose(visual_body, Pose(point=[0, 2*spacing, 0]))
    set_color(visual_body, pp.BLUE)
    LOGGER.info('visual cloned body')
    inspect_data(visual_body, visual_empty=True, collision_empty=True)

    # ! Double cloning will fail because CollisionShapeData's filename is erased to `unknown file`
    with pytest.raises(pb.error) as excinfo:
        double_collision_body = clone_body(collision_body)
    assert 'createCollisionShape failed' in str(excinfo)

    # ! both collison and visual data are empty here
    double_visual_body = clone_body(visual_body)
    inspect_data(double_visual_body, visual_empty=True, collision_empty=True)

    pp.disconnect()


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


@pytest.mark.parametrize("urdf_path",[
    # os.path.join(os.path.dirname(__file__), 'test_data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf'),
    # os.path.join(os.path.dirname(__file__), 'test_data', 'c1', 'urdf', 'c1.urdf'),
    os.path.join(os.path.dirname(__file__), 'test_data', 'link_4.obj'),
    # os.path.join(os.path.dirname(__file__), 'test_data', 'link_4.stl'),
])
@pytest.mark.vertices_from_rigid
def test_vertices_from_link(viewer, urdf_path):
    eps = 1e-6
    connect(use_gui=viewer)
    with HideOutput():
        if urdf_path.endswith('.urdf'):
            body = load_pybullet(urdf_path, fixed_base=False)
        else:
            body = create_obj(urdf_path)
    # wait_if_gui()

    # _, body_links = pp.expand_links(body)
    body_links = pp.get_all_links(body)
    LOGGER.info(f'body links {body_links}')
    body_name = pp.get_body_name(body)

    vertices_from_links = {}
    for attempt in range(2):
        # LOGGER.debug(f'-- attempt {attempt}')
        for body_link in body_links:
            # LOGGER.debug(f'\tlink {body_link}')
            local_from_vertices = pp.vertices_from_rigid(body, body_link)
            # assert len(local_from_vertices) > 0
            if attempt == 0:
                vertices_from_links[body_link] = local_from_vertices
            # LOGGER.debug('#V {} at {} link {}'.format(len(local_from_vertices), body_name, pp.get_link_name(body, body_link)))

            assert len(vertices_from_links[body_link]) == len(local_from_vertices), \
                'unequal num of vertics at link {}'.format(pp.get_link_name(body, body_link))
            for v1, v2 in zip(local_from_vertices, vertices_from_links[body_link]):
                assert pp.get_distance(v1, v2) < eps

            with LockRenderer():
                for v in local_from_vertices:
                    pp.draw_point(v, size=0.05)
        # wait_if_gui()
        pp.remove_all_debug()

    for attempt in range(3):
        # LOGGER.debug(f'-- attempt {attempt}')
        cloned_body = pp.clone_body(body)
        cloned_body_links = pp.get_all_links(cloned_body)
        set_pose(cloned_body, Pose(point=np.random.random(3)*3.0))
        pp.set_color(cloned_body, pp.YELLOW)
        wait_if_gui('Cloned body')
        for body_link in cloned_body_links:
            # LOGGER.debug(f'\tlink {body_link}')
            local_from_vertices = pp.vertices_from_rigid(cloned_body, body_link)
            LOGGER.debug('#V {} at {} link {}'.format(len(local_from_vertices), body_name, pp.get_link_name(cloned_body, body_link)))

            assert len(vertices_from_links[body_link]) == len(local_from_vertices), \
                'unequal num of vertics at link {}'.format(pp.get_link_name(cloned_body, body_link))
            for v1, v2 in zip(local_from_vertices, vertices_from_links[body_link]):
                assert pp.get_distance(v1, v2) < eps

            with LockRenderer():
                for v in local_from_vertices:
                    pp.draw_point(v, size=0.05)
        wait_if_gui()
        pp.remove_all_debug()

    pp.disconnect()


@pytest.mark.vertices_from_link_geom
def test_vertices_from_link_geometry(viewer, robot_path, clamp_urdf_path):
    here = os.path.dirname(__file__)
    mesh_path = os.path.join(here, 'test_data', 'duck.obj')
    connect(use_gui=viewer)
    with HideOutput():
        bodies = {}
        bodies['box'] = pp.create_box(1,1,1, color=apply_alpha(pp.RED, 0.2))
        bodies['cylinder'] = pp.create_cylinder(0.5, 3, color=apply_alpha(pp.GREEN, 0.2))
        bodies['capsule'] = pp.create_capsule(0.5, 3, color=apply_alpha(pp.BLUE, 0.2))
        bodies['sphere'] = pp.create_sphere(0.5, color=apply_alpha(pp.TAN, 0.2))
        bodies['duck'] = pp.create_obj(mesh_path, color=apply_alpha(pp.TAN, 0.2), scale=5e-3)
        bodies['robot'] = load_pybullet(robot_path, fixed_base=True, scale=1.2)
        bodies['clamp'] = load_pybullet(clamp_urdf_path, fixed_base=True, scale=1.5)

    count = 0
    for name, orig_body in bodies.items():
        LOGGER.debug(name)
        count += 1
        for i, body in enumerate([orig_body, clone_body(orig_body)]):
            set_pose(body, Pose(point=[i*2,count*2,0]))
            body_vertices_from_link = pp.get_body_collision_vertices(body)
            body_aabb = pp.get_aabb(body)
            pp.draw_aabb(body_aabb)
            if name not in ['robot', 'clamp']:
                assert len(body_vertices_from_link) > 0
            with LockRenderer():
                for link, verts in body_vertices_from_link.items():
                    for v in verts:
                        # * make sure mesh vertices are loaded in the right scale
                        assert pp.aabb_contains_point(v, body_aabb)
                        pp.draw_point(v, color=pp.BLUE, size=0.1)
                    if verts:
                        orig_extent = pp.get_aabb_extent(body_aabb)
                        point_aabb = pp.aabb_from_points(verts)
                        new_extent = pp.get_aabb_extent(point_aabb)
                        pp.draw_aabb(point_aabb)
                        # ! pybullet's AABB is a bit noisy sometimes, with a safety margin added
                        print(np.abs(new_extent-orig_extent))
                        # if name != 'capsule':
                        #     # ! capsule's vertices do not contain the cap part
                        #     assert np.allclose(new_extent, orig_extent, atol=np.max(orig_extent)/10)
            wait_if_gui()
            pp.remove_all_debug()

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

