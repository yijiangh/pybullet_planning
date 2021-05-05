Contributor's Guide
===================

Contributions are always welcome and greatly appreciated!

Code contributions
------------------

We love pull requests from everyone! Here's a quick guide to improve the code:

1. Fork `the repository <https://github.com/yijiangh/pybullet_planning>`_ and clone the fork.
2. Create a virtual environment using your tool of choice (e.g. ``virtualenv``, ``conda``, etc).
3. Install development dependencies:

::

    pip install -r requirements-dev.txt

4. Make sure all tests pass:

::

    invoke test

5. Start making your changes to the **master** branch (or branch off of it).
6. Make sure all tests still pass:

::

    invoke test

7. Add yourself to ``AUTHORS.rst``.
8. Commit your changes and push your branch to GitHub.
9. Create a `pull request <https://help.github.com/articles/about-pull-requests/>`_ through the GitHub website.


During development, use `pyinvoke <http://docs.pyinvoke.org/>`_ tasks on the
command line to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke``: Show available tasks.

Releasing this project
----------------------

Ready to release a new version of **pybullet_planning**? Here's how to do it:

* We use `semver <https://semver.org/>`_, i.e. we bump versions as follows:

  * ``patch``: bugfixes.
  * ``minor``: backwards-compatible features added.
  * ``major``: backwards-incompatible changes.

* Update the ``CHANGELOG.rst`` with all novelty!
* Ready? Release everything in one command:

::

    invoke release [patch|minor|major]
    # with -b to bump version

* Celebrate! 💃

Documentation improvements
--------------------------

We could always use more documentation, whether as part of the
introduction/examples/usage documentation or API documentation in docstrings.

Documentation is written in `reStructuredText <http://docutils.sourceforge.net/rst.html>`_
and use `Sphinx <http://sphinx-doc.org/index.html>`_ to generate the HTML output.

Once you made the documentation changes locally, run the documentation generation::

    invoke docs


Bug reports
-----------

When `reporting a bug <https://github.com/yijiangh/pybullet_planning/issues>`_
please include:

    * Operating system name and version.
    * Python version.
    * Any details about your local setup that might be helpful in troubleshooting.
    * Detailed steps to reproduce the bug.

Feature requests and feedback
-----------------------------

The best way to send feedback is to file an issue on
`Github <https://github.com/yijiangh/pybullet_planning/issues>`_. If you are proposing a feature:

* Explain in detail how it would work.
* Keep the scope as narrow as possible, to make it easier to implement.
