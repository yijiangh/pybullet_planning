
Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

Unreleased
----------

**Added**

* diagnosis collision checking function to help visualizing the collision bodies' information
* add `workspace_bodies` to the `get_collision_fn` to check collisions with the obstacles specified in a URDF file.
* move `ik_interface` module from application side to this repo, since it's "universal" for fixed-end robot. Might need to add a separete one for robots with moving base later.

**Changed**

**Removed**

**Fixed**

* `utils.numeric_sample.randomize`: `random.shuffle` cannot operate on a `range` in py 3.x. Enforced conversion to `list` to fix it.

**Deprecated**

0.0.1
-------

**Added**

* Initial version
* Divide the original `utils.py` file into separate modules
* Modules cycle dependency resolved.

