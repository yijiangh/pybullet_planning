import pytest
from pybullet_planning import connect, wait_if_gui, disconnect, create_plane

# @pytest.mark.wip_connect
def test_connect(viewer):
    connect(use_gui=viewer, shadows=False, color=[1,0,0], width=500, height=300)
    create_plane(normal=[0,0,1])
    wait_if_gui()
    disconnect()

    connect(use_gui=viewer, shadows=True, width=1024, height=768)
    create_plane(normal=[1,1,1])
    wait_if_gui()
    disconnect()
