"""
********************************************************************************
primitives
********************************************************************************

These modules are transported from `kuka_primitives`, `pr2_primitives` and `pr2_utils` from
`ss-pybullet <https://github.com/caelan/ss-pybullet/tree/master/pybullet_tools>`_.

.. currentmodule:: pybullet_planning.primitives

Grasp
------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    get_top_grasps
    get_side_grasps
    get_side_cylinder_grasps
    get_edge_cylinder_grasps

Convenient classes
------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    EndEffector

"""

from __future__ import absolute_import

from .grasp import *

__all__ = [name for name in dir() if not name.startswith('_')]
