import pytest
from numpy.testing import assert_almost_equal
from pybullet_planning import connect, wait_for_user, has_gui
from pybullet_planning import get_pose, draw_pose, set_color, set_pose, Pose, STATIC_MASS

from pybullet_planning import create_box, create_cylinder, create_capsule, create_plane
from pybullet_planning import approximate_as_prism, approximate_as_cylinder

# @pytest.mark.wip_shape
def test_box(viewer):
    connect(viewer)
    w = .1
    l = .2
    h = .3
    body = create_box(w, l, h)
    set_color(body, (1,0,0,0.2))
    if has_gui():
        draw_pose(get_pose(body))
        wait_for_user()
    center, bounding_extent = approximate_as_prism(body)
    assert_almost_equal(center, [0,0,0], decimal=6)
    assert_almost_equal(bounding_extent, [w, l, h], decimal=6)

# @pytest.mark.wip_shape
def test_cylinder(viewer):
    connect(viewer)
    radius = 0.02
    h = .3
    body = create_cylinder(radius, h)
    set_color(body, (1,0,0,0.2))
    draw_pose(get_pose(body))
    if has_gui():
        draw_pose(get_pose(body))
        wait_for_user()
    center, (diameter, height) = approximate_as_cylinder(body)
    assert_almost_equal(center, [0,0,0], decimal=6)
    assert_almost_equal([diameter, height], [2*radius, h], decimal=6)
