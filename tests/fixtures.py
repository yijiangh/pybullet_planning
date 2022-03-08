import os, pytest
import pybullet_planning as pp
from pybullet_planning import apply_alpha

here = os.path.dirname(__file__)
FILE_FROM_NAME = {
    'robot' : os.path.join(here, 'test_data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf'),
    'workspace' : os.path.join(here, 'test_data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf'),
    'clamp' : os.path.join(here, 'test_data', 'c1', 'urdf', 'c1.urdf'),
    'gripper' : os.path.join(here, 'test_data', 'dms_bar_gripper.obj'),
    'duck' : os.path.join(here, 'test_data', 'duck.obj'),
    'link4_stl' : os.path.join(here, 'test_data', 'link_4.obj'),
}

@pytest.fixture
def robot_path():
    return FILE_FROM_NAME['robot']

@pytest.fixture
def workspace_path():
    return FILE_FROM_NAME['workspace']

@pytest.fixture
def clamp_urdf_path():
    return FILE_FROM_NAME['clamp']

@pytest.fixture
def ee_path():
    return FILE_FROM_NAME['gripper']

@pytest.fixture
def duck_obj_path():
    return FILE_FROM_NAME['duck']

# @pytest.fixture
# def abb_path():
#     return FILE_FROM_NAME['abb']

@pytest.fixture
def link4_stl_path():
    return FILE_FROM_NAME['link4_stl']

################################

def load_body_lib():
    body_lib = {}
    body_lib['box'] = pp.create_box(1,1,1, color=apply_alpha(pp.RED, 0.2))
    body_lib['cylinder'] = pp.create_cylinder(0.5, 3, color=apply_alpha(pp.GREEN, 0.2))
    body_lib['capsule'] = pp.create_capsule(0.5, 3, color=apply_alpha(pp.BLUE, 0.2))
    body_lib['sphere'] = pp.create_sphere(0.5, color=apply_alpha(pp.TAN, 0.2))
    body_lib['duck'] = pp.create_obj(FILE_FROM_NAME['duck'], color=apply_alpha(pp.TAN, 0.2), scale=5e-3)
    body_lib['robot'] = pp.load_pybullet(FILE_FROM_NAME['robot'], fixed_base=True, scale=1.2)
    body_lib['clamp'] = pp.load_pybullet(FILE_FROM_NAME['clamp'], fixed_base=True, scale=1.5)
    body_lib['link4_stl'] = pp.load_pybullet(FILE_FROM_NAME['link4_stl'], fixed_base=True, scale=0.8)
    return body_lib

    # body_lib['abb'] = pp.load_pybullet(FILE_FROM_NAME['abb'], fixed_base=True, scale=1.2)
