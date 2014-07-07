^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometric_shapes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-07-07)
------------------
* update distro for travis testing. precise:=trusty
* update to use debian console_bridge dependency. https://github.com/ros/rosdistro/issues/4633
* Contributors: Ioan A Sucan, Tully Foote

0.4.0 (2014-06-24)
------------------
* update usage of console_bridge to deal with version in Trusty
* Merge pull request `#13 <https://github.com/ros-planning/geometric_shapes/issues/13>`_ from ros-planning/testing-in-travis
  Run local and moveit_core tests in Travis builds.
* Merge pull request `#18 <https://github.com/ros-planning/geometric_shapes/issues/18>`_ from dirk-thomas/hydro-devel
  fix configure config.h.in when paths contain spaces fix `#9 <https://github.com/ros-planning/geometric_shapes/issues/9>`_
* Run local and moveit_core tests in Travis builds.
* Contributors: Acorn, Dave Hershberger, Dirk Thomas, Ioan A Sucan, William Woodall

0.3.8 (2014-02-25)
------------------
* fix how we find eigen
* Contributors: Ioan Sucan

0.3.7 (2014-02-23)
------------------
* add build dep so we can find eigen, build fixes
* Contributors: Ioan A Sucan, Scott K Logan

0.3.6 (2014-01-31)
------------------
* Use assimp-dev dep for building
* Remove stray IS_ASSIMP3 define
* Invert Assimp version detect logic for greater accuracy
* Better feature detection for assimp version
* added travis support
* check for CATKIN_ENABLE_TESTING
* Contributors: Dave Hershberger, Ioan A Sucan, Lukas Bulwahn, Scott K Logan

0.3.5 (2013-09-23)
------------------
* Fix syntax error.
* white space fixes (tabs are now spaces)
* add comments for shape definitions
