"""
********************************************************************************
interfaces.robots
********************************************************************************

.. currentmodule:: pybullet_planning.interfaces.robots

TODO: module description

Body
--------------

.. currentmodule:: pybullet_planning.interfaces.robots

.. autosummary::
    :toctree: generated/
    :nosignatures:

    vertices_from_rigid


Body Approximation
-------------------

.. currentmodule:: pybullet_planning.interfaces.robots

.. autosummary::
    :toctree: generated/
    :nosignatures:

    approximate_as_prism
    approximate_as_cylinder

"""

from .joint import *
from .link import *
from .dynamics import *
from .body import *
from .collision import *

__all__ = [name for name in dir() if not name.startswith('_')]
