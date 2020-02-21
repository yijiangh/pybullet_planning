import os
import random
import numpy as np

import pytest
from numpy.testing import assert_almost_equal
from pybullet_planning import connect, wait_for_user, has_gui, HideOutput
from pybullet_planning import get_pose, draw_pose, set_color, set_pose, Pose, STATIC_MASS
from pybullet_planning import create_cylinder, link_from_name, load_pybullet, approximate_as_cylinder, point_from_pose, \
    multiply, quat_from_euler, unit_pose, Euler, Point, end_effector_from_body, remove_handles, set_camera_pose, \
    body_from_end_effector, invert, create_obj

from pybullet_planning import EndEffector, get_side_cylinder_grasps

@pytest.fixture
def robot_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'universal_robot', 'ur_description', 'urdf', 'ur5_w_gripper.urdf')

@pytest.fixture
def ee_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'gripper', 'urdf', 'gripper.urdf')

@pytest.fixture
def ee_link_names():
    TOOL_LINK = 'eef_tcp_frame'
    EE_LINK = 'eef_base_link'
    return TOOL_LINK, EE_LINK

@pytest.fixture
def obj_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'duck.obj')

def focus_camera(centroid):
    camera_offset = 0.1 * np.array([1, 1, 1])
    set_camera_pose(camera_point=centroid + camera_offset, target_point=centroid)

@pytest.mark.wip_grasp
def test_side_grasp(viewer, ee_path, ee_link_names, obj_path):
    connect(use_gui=viewer)
    focus_camera([0,0,0])
    with HideOutput():
        ee = load_pybullet(ee_path, fixed_base=False)
        # move it away for a clean scene
        set_pose(ee, Pose(point=Point(-2,0,0)))
    tool_link_name, ee_link_name = ee_link_names

    end_effector = EndEffector(ee, ee_link=link_from_name(ee, ee_link_name),
                               tool_link=link_from_name(ee, tool_link_name),
                               visual=False, collision=True)
    draw_pose(end_effector.get_tool_pose())

    obj_body = create_obj(obj_path, scale=1e-3, color=(0,0,1,0.3))
    # set_color(obj_body, )
    obj_pose = get_pose(obj_body)

    # tool_pose = Pose(euler=Euler(yaw=np.pi/2))
    for grasp in get_side_cylinder_grasps(obj_body):
        print(grasp)
        world_from_ee = end_effector_from_body(obj_pose, grasp)
        end_effector.set_pose(world_from_ee)
        wait_for_user()
