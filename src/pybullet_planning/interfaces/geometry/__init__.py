"""
********************************************************************************
interfaces.geometry
********************************************************************************

.. currentmodule:: pybullet_planning.interfaces.geometry

TODO: module description

Main Types
--------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    AABB

Bounding box operations
-----------------------

.. currentmodule:: pybullet_planning.interfaces.geometry.bounding_box

.. autosummary::
    :toctree: generated/
    :nosignatures:

    aabb_from_points
    aabb_union
    get_bodies_in_region

"""

from .mesh import *
from .polygon import *
from .bounding_box import *
from .pointcloud import *
from .camera import *

__all__ = [name for name in dir() if not name.startswith('_')]
