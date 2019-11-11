import pytest
from pybullet_planning import connect, wait_for_user, disconnect

def test_connect():
    connect(use_gui=False)
    disconnect()
