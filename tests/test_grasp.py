import os

import pytest
from numpy.testing import assert_almost_equal
from pybullet_planning import connect, wait_for_user, has_gui, HideOutput
from pybullet_planning import get_pose, draw_pose, set_color, set_pose, Pose, STATIC_MASS
from pybullet_planning import create_box, link_from_name, load_pybullet

from pybullet_planning import EndEffector

@pytest.fixture
def robot_path():
    here = os.path.dirname(__file__)
    return os.path.join(here, 'test_data', 'universal_robot', 'ur_description', 'urdf', 'ur5_w_gripper.urdf')

@pytest.fixture
def ee_link_names():
    TOOL_LINK = 'eef_tcp_frame'
    EE_LINK = 'eef_base_link'
    return TOOL_LINK, EE_LINK

@pytest.mark.wip_grasp
def test_side_grasp(viewer, robot_path, ee_link_names):
    connect(use_gui=viewer)
    with HideOutput():
        robot = load_pybullet(robot_path, fixed_base=True)
    tool_link_name, ee_link_name = ee_link_names

    end_effector = EndEffector(robot, ee_link=link_from_name(robot, ee_link_name),
                               tool_link=link_from_name(robot, tool_link_name),
                               visual=False, collision=True)

    draw_pose(end_effector.get_tool_pose())

    # w = .1
    # l = .2
    # h = .3
    # body = create_box(w, l, h)
    # wait_for_user()

