import os, pytest
import numpy as np
import pybullet_planning as pp
from pybullet_planning import LOGGER, wait_if_gui
from pybullet_planning import load_pybullet, connect, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, wait_if_gui, apply_alpha
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints
from pybullet_planning import dump_world, set_pose, set_color
from pybullet_planning import clone_body, create_box, read_obj
from pybullet_planning import Pose

def inspect_data(body, original_aabb=None, collision_empty=False, visual_empty=False, verbose=False):
    links = pp.get_all_links(body)
    for i, link in enumerate(links):
        collision_data = pp.get_collision_data(body, link)
        visual_data = pp.get_visual_data(body, link)
        if verbose:
            LOGGER.debug(f'Link {i}')
            LOGGER.debug('collsion: {}'.format(collision_data))
            LOGGER.debug('visual: {}'.format(visual_data))
        if collision_empty:
            assert len(collision_data) == 0
        if visual_empty:
            assert len(visual_data) == 0
    if original_aabb:
        orig_extent = pp.get_aabb_extent(original_aabb)
        new_extent = pp.get_aabb_extent(pp.get_aabb(body))
        pp.draw_aabb(original_aabb)
        pp.draw_aabb(pp.get_aabb(body))
        wait_if_gui()
        # ! pybullet's AABB is a bit noisy sometimes, with a safety margin added
        assert np.allclose(new_extent, orig_extent, atol=np.max(orig_extent)/10)

@pytest.mark.clone_body_single_obj
@pytest.mark.parametrize("body_name",[
    'obj',
    'box',
    ])
def test_clone_body_single_obj(viewer, body_name, ee_path, print_debug):
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
    set_color(cloned_body, pp.apply_alpha(pp.GREEN, 0.2))
    set_pose(cloned_body, Pose(point=[0, 0, spacing]))
    LOGGER.info('both cloned body')
    inspect_data(cloned_body, original_aabb, verbose=print_debug)

    collision_body = clone_body(body, visual=False)
    set_color(collision_body, pp.RED)
    set_pose(collision_body, Pose(point=[0, 1*spacing, 0]))
    LOGGER.info('collision cloned body')
    inspect_data(collision_body, original_aabb, verbose=print_debug)

    # TODO fix these
    # visual_body = clone_body(body, collision=False)
    # set_pose(visual_body, Pose(point=[0, 2*spacing, 0]))
    # set_color(visual_body, pp.BLUE)
    # LOGGER.info('visual cloned body')
    # inspect_data(visual_body)

    # double_collision_body = clone_body(collision_body)
    # set_pose(double_collision_body, Pose(point=[1*spacing, 1*spacing, 0]))
    # set_color(double_collision_body, pp.YELLOW)
    # LOGGER.info('double collision cloned body')
    # inspect_data(double_collision_body, original_aabb)

    # double_visual_body = clone_body(visual_body)
    # set_pose(double_visual_body, Pose(point=[1*spacing, 2*spacing, 0]))
    # set_color(double_visual_body, pp.BROWN)
    # LOGGER.info('double visual cloned body')
    # inspect_data(double_visual_body)

    pp.reset_simulation()
    pp.disconnect()

@pytest.mark.clone_body_urdf
@pytest.mark.parametrize("body_name",[
    'workspace',
    'robot',
    'urdf_link_multi_mesh',
    ])
def test_clone_body_urdf(viewer, body_name, robot_path, clamp_urdf_path, workspace_path, print_debug):
    connect(use_gui=viewer)
    with HideOutput():
        if body_name == 'workspace':
            body = load_pybullet(workspace_path, fixed_base=True, scale=0.1)
        elif body_name == 'robot':
            body = load_pybullet(robot_path, fixed_base=True, scale=0.2)
        elif body_name == 'urdf_link_multi_mesh':
            body = load_pybullet(clamp_urdf_path, fixed_base=True)
        set_color(body, pp.GREY)

    # TODO PyBullet does not return any VisualShapeData at all when viewer is on
    LOGGER.info('original body')
    original_aabb = pp.get_aabb(body)
    inspect_data(body, original_aabb, verbose=print_debug)

    spacing = 1.5
    # CollisionShapeData's mesh filename will be turned into `unknown file`
    # with VisualShapeData pointing to collision mesh filename.
    LOGGER.info('both cloned body')
    cloned_body = clone_body(body)
    set_color(cloned_body, pp.GREEN)
    set_pose(cloned_body, Pose(point=[0, 0, spacing]))
    inspect_data(cloned_body, original_aabb, verbose=print_debug)

    # ! both collision and visual data will be cloned, behavior same as with above,
    # when `collision=True, visual=True`
    LOGGER.info('collision cloned body')
    collision_body = clone_body(body, visual=False)
    set_color(collision_body, pp.RED)
    set_pose(collision_body, Pose(point=[0, 1*spacing, 0]))
    inspect_data(collision_body, original_aabb, verbose=print_debug)

    # TODO fix these
    # # ! both collison and visual data are empty here
    # LOGGER.info('visual cloned body')
    # visual_body = clone_body(body, collision=False)
    # set_pose(visual_body, Pose(point=[0, 2*spacing, 0]))
    # set_color(visual_body, pp.BLUE)
    # inspect_data(visual_body, visual_empty=has_gui(), collision_empty=True)

    # # ! Double cloning will fail because CollisionShapeData's filename is erased to `unknown file`
    # LOGGER.info('double collision cloned body')
    # # with pytest.raises(pb.error) as excinfo:
    # double_collision_body = clone_body(collision_body)
    # # assert 'createCollisionShape failed' in str(excinfo)
    # set_pose(double_collision_body, Pose(point=[1*spacing, 1*spacing, 0]))
    # set_color(double_collision_body, pp.YELLOW)
    # inspect_data(double_collision_body, collision_empty=pp.has_gui())

    # # ! both collison and visual data are empty here
    # LOGGER.info('double visual cloned body')
    # double_visual_body = clone_body(visual_body)
    # set_pose(double_visual_body, Pose(point=[1*spacing, 2*spacing, 0]))
    # set_color(double_visual_body, pp.BROWN)
    # inspect_data(double_visual_body, visual_empty=has_gui(), collision_empty=True)

    pp.reset_simulation()
    pp.disconnect()
