"""
********************************************************************************
interfaces.robots
********************************************************************************

.. currentmodule:: pybullet_planning.interfaces.robots

TODO: module description

Body
--------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    vertices_from_rigid

Collision checking
--------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    get_collision_fn
    get_floating_body_collision_fn

Body Approximation
-------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    approximate_as_prism
    approximate_as_cylinder

Dynamics
-------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    DynamicsInfo
    set_static

Verbose
-------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    dump_body
    dump_world

"""

from .joint import *
from .link import *
from .dynamics import *
from .body import *
from .collision import *

__all__ = [name for name in dir() if not name.startswith('_')]
