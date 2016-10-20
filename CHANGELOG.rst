^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometric_shapes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.5 (2016-10-20)
------------------
* [fix] mesh with too many vertices (`#39 <https://github.com/ros-planning/geometric_shapes/issues/39>`_) (`#60 <https://github.com/ros-planning/geometric_shapes/issues/60>`_)
* [fix] gcc6 build error (`#56 <https://github.com/ros-planning/geometric_shapes/issues/55>`_)
* [fix] Clear root transformation on imported Collada meshes. `#52 <https://github.com/ros-planning/geometric_shapes/issues/52>`_
* [fix] incorrect hint always sent to Assimp, improved STL reading (`#24 <https://github.com/ros-planning/geometric_shapes/issues/24>`_)
* [fix] append cmake module path instead of prepending. Fixes `#22 <https://github.com/ros-planning/geometric_shapes/issues/22>`_
* [fix] FindQhull with non-debian systems (`#30 <https://github.com/ros-planning/geometric_shapes/issues/30>`_)
* [improve] relax mesh containment test (`#58 <https://github.com/ros-planning/geometric_shapes/issues/58>`_)
* [maintenance] Switch boost::shared_ptr to std::shared_ptr. `#57 <https://github.com/ros-planning/geometric_shapes/pull/57>`_
* [maintenance] Fix travis for Indigo (`#53 <https://github.com/ros-planning/geometric_shapes/issues/53>`_)
* [maintenance] catkin_lint and cleanup `#43 <https://github.com/ros-planning/geometric_shapes/issues/43>`_
* [maintenance] add notice that project will be built in Release mode
* [maintenance] cmake cleanup and improvement (`#35 <https://github.com/ros-planning/geometric_shapes/issues/35>`_ and others)
* Contributors: Dave Coleman, Ioan A Sucan, Isaac I.Y. Saito, Jochen Sprickerhof, Jorge Santos Simon, Lukas Bulwahn, Maarten de Vries, Michael Goerner,

0.4.4 (2016-03-06)
------------------
* Merge pull request `#37 <https://github.com/ros-planning/geometric_shapes/issues/37>`_ from corot/indigo-devel
  Fix issue `#28 <https://github.com/ros-planning/geometric_shapes/issues/28>`_ on small radius cylinders
* Contributors: Dave Coleman, Jorge Santos Simon

0.4.3 (2015-04-30)
------------------
* add functions for better display of convex meshes
* produce actual triangles for qhull mesh
* Fixed inverted scale for convex meshes inside check
* Contributors: Christian Dornhege, Michael Ferguson

0.4.2 (2015-04-22)
------------------
* PR `#32 <https://github.com/ros-planning/geometric_shapes/issues/32>`_
  Merge shape_tools package into geometric shapes
* PR `#33 <https://github.com/ros-planning/geometric_shapes/issues/33>`_
  Add run_depend on visualization_msgs
* PR `#26 <https://github.com/ros-planning/geometric_shapes/issues/26>`_
  Prevent every mesh generation opening a new file handle.
* Contributors: Christian Dornhege, Dave Coleman, Jochen Sprickerhof, Michael Ferguson, Steven Peters

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
