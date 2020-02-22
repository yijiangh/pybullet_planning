import math
import random
import numpy as np

from pybullet_planning.interfaces import Pose, Point, Euler, unit_pose, point_from_pose, multiply, quat_from_euler, invert
from pybullet_planning.interfaces import approximate_as_prism, approximate_as_cylinder


def get_top_grasps(body, under=False):
    raise NotImplementedError()


def get_side_grasps(body, under=False):
    raise NotImplementedError()


def get_side_cylinder_grasps(body, tool_pose=unit_pose(), body_pose=unit_pose(), reverse_grasp=False,
    safety_margin_length=0.0):
    """generate side grasp using the body's bounding cylinder.

    The generated grasp should be used as: ``world_from_body = world_from_gripper * grasp``

    The image below shows the default gripper axis convention.

    .. image:: ../images/cylinder_grasp_convention.png
        :align: center

    Parameters
    ----------
    body : int
        body id for the body being grasped
    tool_pose : Pose, optional
        additional default_ee_from_customized_ee, by default unit_pose()
    body_pose : Pose, optional
        the body's current pose, used for computing bounding box, by default unit_pose()
    reverse_grasp : bool, optional
        generate flipped gripper for pi-symmetry around the gripper's z axis, by default False
    safety_margin_length : float, optional
        safety margin along the gripper's y axis, i.e. the ``top_offset`` shown in the picture, by default 0.0

    Yields
    -------
    Pose
        gripper_from_body
    """
    center, (_, height) = approximate_as_cylinder(body, body_pose=body_pose)
    assert safety_margin_length <= height/2, \
    'safety margin length must be smaller than half of the bounding cylinder\'s height {}'.format(height/2)
    object_from_center = Pose(center - point_from_pose(body_pose))

    # rotate the cylinder's frame to make x axis align with the longitude axis
    longitude_x = Pose(euler=Euler(pitch=np.pi/2))
    while True:
        # rotational symmetry around the longitude axis
        theta = random.uniform(0, 2*np.pi)
        rotate_around_x_axis = Pose(euler=Euler(theta, 0, 0))
        # translation along the longitude axis
        slide_dist = random.uniform(-height/2+safety_margin_length, height/2-safety_margin_length)
        translate_along_x_axis = Pose(point=Point(slide_dist,0,0))

        for j in range(1 + reverse_grasp):
            # the base pi/2 is to make y align with the longitude axis, conforming to the convention (see image in the doc)
            # flip the gripper, gripper symmetry
            rotate_around_z = Pose(euler=[0, 0, math.pi/2 + j * math.pi])

            object_from_gripper = multiply(object_from_center, longitude_x, translate_along_x_axis, rotate_around_x_axis, \
                rotate_around_z, tool_pose)
            yield invert(object_from_gripper)
