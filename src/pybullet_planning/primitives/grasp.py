import math
import random
import numpy as np

from pybullet_planning.interfaces import Pose, Euler, unit_pose, point_from_pose, multiply, quat_from_euler, invert
from pybullet_planning.interfaces import approximate_as_prism, approximate_as_cylinder


def get_top_grasps(body, under=False):
    raise NotImplementedError()

def get_side_grasps(body, under=False):
    raise NotImplementedError()

def get_side_cylinder_grasps(body, tool_pose=unit_pose(), body_pose=unit_pose(), reverse_grasp=False):
    # TODO: consider diameter
    # TODO: slide along height (longitude axis)
    center, (diameter, height) = approximate_as_cylinder(body, body_pose=body_pose)
    print(center, (diameter, height))
    translate_center = Pose(point_from_pose(body_pose)-center)

    # rotate the cylinder's frame to make x axis align with the longitude axis
    longitude_x = Pose(euler=Euler(roll=np.pi/2))
    while True:
        theta = random.uniform(0, 2*np.pi)
        # rotational symmetry around the longitude axis
        rotate_around_x_axis = Pose(euler=Euler(0, theta, 0))
        for j in range(1 + reverse_grasp):
            # flip the gripper, gripper symmetry
            # rotate_around_z_axis = Pose(euler=Euler(0, 0, j*np.pi))
            object_from_gripper = multiply(translate_center, longitude_x, rotate_around_x_axis, tool_pose)
            yield invert(object_from_gripper)
