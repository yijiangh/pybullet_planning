"""
********************************************************************************
primitives
********************************************************************************

These modules are transported from `kuka_primitives`, `pr2_primitives` and `pr2_utils` from
`ss-pybullet <https://github.com/caelan/ss-pybullet/tree/master/pybullet_tools>`_.

.. currentmodule:: pybullet_planning.primitives

Grasp
------

In ``pybullet_planning``, a grasp is modeled as an operator on the object frame, i.e.
``world_from_object = world_from_gripper * grasp``, where ``grasp = gripper_from_object``.

We provide some simple grasp generator based on bounding geometry (box, cylinder). Inside these
generators' implementation, however, the modeling start from the object frame to the gripper's
frame to obtain ``object_from_gripper`` and then we return its inverse ``gripper_from_object``.
(Yijiang thinks this is more straightforward to think about.)

.. currentmodule:: pybullet_planning.primitives.grasp_gen

.. autosummary::
    :toctree: generated/
    :nosignatures:

    get_side_cylinder_grasps

Convenient classes
------------------

.. currentmodule:: pybullet_planning.primitives.trajectory

.. autosummary::
    :toctree: generated/
    :nosignatures:

    EndEffector

"""

from __future__ import absolute_import

from .grasp_gen import *
from .trajectory import *

__all__ = [name for name in dir() if not name.startswith('_')]
