import os, pytest
from pybullet_planning.interfaces.robots.joint import set_joint_positions
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

from fixtures import load_body_lib

@pytest.mark.vertices_from_link_geom
def test_vertices_from_link_geometry(viewer):
    connect(use_gui=viewer)
    LOGGER.info(pp.INFO_FROM_BODY)
    with HideOutput():
        bodies = load_body_lib()
    count = 0
    for name, orig_body in bodies.items():
        LOGGER.debug(name)
        count += 1
        for i, body in enumerate([orig_body]): #, clone_body(orig_body)]):
            set_color(body, pp.apply_alpha(pp.GREY, 0.2))
            set_pose(body, Pose(point=[i*2,count*2,0]))

            joints = get_movable_joints(body)
            if len(joints) > 0:
                sample_fn = pp.get_sample_fn(body, joints)
            attempts = 10 if len(joints) > 0 else 1
            for _ in range(attempts):
                if len(joints) > 0:
                    set_joint_positions(body, joints, sample_fn())
                body_vertices_from_link = pp.get_body_collision_vertices(body)
                body_aabb = pp.get_aabb(body)
                pp.draw_aabb(body_aabb)
                # wait_if_gui('main aabb')

                if name not in ['robot', 'clamp', 'abb']:
                    assert len(body_vertices_from_link) > 0
                all_link_verts= []
                for link, verts in body_vertices_from_link.items():
                    with LockRenderer():
                        for v in verts:
                            # * make sure mesh vertices are loaded in the right scale
                            assert pp.aabb_contains_point(v, body_aabb)
                            pp.draw_point(v, color=pp.BLUE, size=0.1)
                    all_link_verts.extend(verts)
                # wait_if_gui('Draw vertices')

                if all_link_verts:
                    orig_extent = pp.get_aabb_extent(body_aabb)
                    point_aabb = pp.aabb_from_points(all_link_verts)
                    new_extent = pp.get_aabb_extent(point_aabb)
                    pp.draw_aabb(point_aabb)

                    # ! pybullet's AABB is a bit noisy sometimes, with a safety margin added
                    print(np.abs(new_extent-orig_extent))
                    if name != 'capsule':
                        # ! capsule's vertices do not contain the cap part
                        assert np.allclose(new_extent, orig_extent, atol=np.max(orig_extent)/5)
                    # wait_if_gui()
            pp.remove_all_debug()

    pp.reset_simulation()
    pp.disconnect()

@pytest.mark.parametrize("urdf_path",[
    os.path.join(os.path.dirname(__file__), 'test_data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf'),
    os.path.join(os.path.dirname(__file__), 'test_data', 'c1', 'urdf', 'c1.urdf'),
    os.path.join(os.path.dirname(__file__), 'test_data', 'link_4.obj'),
    os.path.join(os.path.dirname(__file__), 'test_data', 'link_4.stl'),
])
@pytest.mark.vertices_from_rigid
def test_vertices_from_link(viewer, urdf_path):
    eps = 1e-6
    connect(use_gui=viewer)
    with HideOutput():
        body = load_pybullet(urdf_path, fixed_base=False)

    body_links = pp.get_all_links(body)
    body_name = pp.get_body_name(body)
    # LOGGER.info(f'body links {body_links}')

    body_aabb = pp.get_aabb(body)
    pp.draw_aabb(body_aabb)

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

            for v in local_from_vertices:
                assert pp.aabb_contains_point(v, body_aabb)

            with LockRenderer():
                for v in local_from_vertices:
                    pp.draw_point(v, size=0.05)
        wait_if_gui()
        pp.remove_all_debug()

    # TODO fix vertices_from_rigid for cloned_body
    # for attempt in range(3):
    #     # LOGGER.debug(f'-- attempt {attempt}')
    #     cloned_body = pp.clone_body(body)
    #     cloned_body_links = pp.get_all_links(cloned_body)
    #     set_pose(cloned_body, Pose(point=np.random.random(3)*3.0))
    #     pp.set_color(cloned_body, pp.YELLOW)
    #     wait_if_gui('Cloned body')
    #     for body_link in cloned_body_links:
    #         # LOGGER.debug(f'\tlink {body_link}')
    #         local_from_vertices = pp.vertices_from_rigid(cloned_body, body_link)
    #         LOGGER.debug('#V {} at {} link {}'.format(len(local_from_vertices), body_name, pp.get_link_name(cloned_body, body_link)))

    #         assert len(vertices_from_links[body_link]) == len(local_from_vertices), \
    #             'unequal num of vertics at link {}'.format(pp.get_link_name(cloned_body, body_link))
    #         for v1, v2 in zip(local_from_vertices, vertices_from_links[body_link]):
    #             assert pp.get_distance(v1, v2) < eps

    #         with LockRenderer():
    #             for v in local_from_vertices:
    #                 pp.draw_point(v, size=0.05)
    #     wait_if_gui()
    #     pp.remove_all_debug()

    pp.disconnect()
