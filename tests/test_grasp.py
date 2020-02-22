import os
import random
import numpy as np

import pytest
from numpy.testing import assert_almost_equal
from pybullet_planning import connect, wait_for_user, has_gui, HideOutput
from pybullet_planning import get_pose, draw_pose, set_color, set_pose, Pose, STATIC_MASS
from pybullet_planning import create_cylinder, link_from_name, load_pybullet, approximate_as_cylinder, point_from_pose, \
    multiply, quat_from_euler, unit_pose, Euler, Point, end_effector_from_body, remove_handles, set_camera_pose, \
    body_from_end_effector, invert, create_obj, interval_generator

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

# @pytest.mark.wip_grasp
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
                               visual=True, collision=True)
    draw_pose(end_effector.get_tool_pose())

    epsilon = 0.1
    angle = np.pi/3
    lower = [-epsilon]*3 + [-angle]*3
    upper = [epsilon]*3 + [angle]*3
    [x, y, z, roll, pitch, yaw] = next(interval_generator(lower, upper))
    random_pose = Pose(point=[x,y,z], euler=Euler(roll=roll, pitch=pitch, yaw=yaw))
    test_obj_poses = [unit_pose(), random_pose]

    obj_body = create_obj(obj_path, scale=1e-3, color=(0,0,1,0.3))

    for obj_pose in test_obj_poses:
        print('Obj pose: ', obj_pose)
        set_pose(obj_body, obj_pose)

        if has_gui():
            # draw bounding cylinder
            draw_pose(obj_pose, length=0.02)
            center, (diameter, height) = approximate_as_cylinder(obj_body, body_pose=obj_pose)
            cy = create_cylinder(diameter/2, height, color=(1,0,0,0.3))
            set_pose(cy, (center, obj_pose[1]))

        # rotational adjustment if needed
        tool_pose = Pose(euler=Euler(yaw=np.pi/2))
        # translation adjustment to make grasp contact point closer to the jaw's center
        # tool_pose = Pose(point=Point(0,0,0.02))
        n_attempts = 10
        grasp_gen = get_side_cylinder_grasps(obj_body, tool_pose=tool_pose, reverse_grasp=True, safety_margin_length=0.005)
        for _ in range(n_attempts):
            handles = []
            grasp = next(grasp_gen)
            world_from_ee = end_effector_from_body(obj_pose, grasp)
            end_effector.set_pose(world_from_ee)
            handles.extend(draw_pose(world_from_ee))
            if has_gui():
                wait_for_user()
            remove_handles(handles)

