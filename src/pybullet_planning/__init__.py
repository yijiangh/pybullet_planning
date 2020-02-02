"""
********************************************************************************
pybullet_planning
********************************************************************************

.. currentmodule:: pybullet_planning

This library is a suite of utility functions to facilitate robotic planning related research on the `pybullet <https://github.com/bulletphysics/bullet3>`_ physics simulation engine.

.. toctree::
    :maxdepth: 1

    pybullet_planning.interfaces
    pybullet_planning.motion_planners
    pybullet_planning.utils

"""

from .__version__ import __author__, __author_email__, __copyright__, __description__, __license__, \
    __title__, __url__, __version__

from .utils import *
from .interfaces import *
from .motion_planners import *

__all__ = ['__author__', '__author_email__', '__copyright__', '__description__', '__license__', '__title__', '__url__', '__version__']
