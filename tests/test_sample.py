import pytest
from pybullet_planning.interfaces import connect, wait_for_user
# from conrob_pybullet import connect, wait_for_user

@pytest.mark.wip
def test_import():
    connect(use_gui=True)
    wait_for_user()
