"""

Python implementations of several robotic motion planners

Sampling-based:
* Probabilistic Roadmap (PRM)
* Rapidly-Exploring Random Tree (RRT)
* RRT-Connect (BiRRT)
* Linear Shortcutting
* MultiRRT
* RRT*

Grid search
* Breadth-First Search (BFS)
* A*


Setup
=====

In order to use this library, ...


Main concepts
=============

Describe typical classes found in project

.. autoclass:: SampleClassName
   :members:


"""

from .rrt_connect import birrt, direct_path
from .lazy_prm import lazy_prm

# __all__ = [name for name in dir() if not name.startswith('_')]
