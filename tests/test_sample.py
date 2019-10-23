import pytest
from pybullet_planning import connect, wait_for_user
from pybullet_planning import create_mesh
from pybullet_planning import load_model
# from conrob_pybullet import connect, wait_for_user

@pytest.mark.wip
def test_import():
    connect(use_gui=True)
    # wait_for_user()
