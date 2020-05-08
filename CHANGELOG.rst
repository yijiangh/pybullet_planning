
Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.


0.2.0
----------

**Changed**

* Changed doc theme to sphinx-rtd-theme, doc hosted on readthedocs
* Changed `motion_planners` to keep updated with https://github.com/caelan/motion-planners

**Added**

* Added `SE3` floating body motion planning
* Added ladder graph cartesian planner

**Removed**

* Removed `requirments.txt`, moved dependencies into `setup.py`

0.1.1
----------

**Fixed**

* Fixed Windows OS "Display not in os.environ" issue in `connect`


0.1.0
----------

**Added**

* diagnosis collision checking function to help visualizing the collision bodies' information
* add `workspace_bodies` to the `get_collision_fn` to check collisions with the obstacles specified in a URDF file.
* move `ik_interface` module from application side to this repo, since it's "universal" for fixed-end robot. Might need to add a separete one for robots with moving base later.
* enable travis ci unit test, collision_fn well tested
* `get_floating_body_collision_fn` to check a body without joints's collision. Associated test added.
* Added `snap_sols` to `kinematics.ik_utils`

**Changed**

* add `extra_disabled_collisions` parameter to `get_collision_fn`, `plan_joint_motion`
* Changed `get_name` to return name without index if name is not `''`

**Removed**

* `get_collision_diagnosis_fn` removed, integrated into the `get_collision_fn`

**Fixed**

* `utils.numeric_sample.randomize`: `random.shuffle` cannot operate on a `range` in py 3.x. Enforced conversion to `list` to fix it.
* Fixed `get_collision_fn` to ignore checking between bodies both specified in attachment and obstacles (prioritize its role as attachment)

**Deprecated**

**TODO**

* add body name for bodies from `create_obj`

**Requested features**

* `clone_body` work for bodies from `create_obj`


0.0.1
-------

**Added**

* Initial version
* Divide the original `utils.py` file into separate modules
* Modules cycle dependency resolved.

