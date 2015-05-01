^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometric_shapes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.9 (2015-04-30)
------------------
* Prevent every mesh generation opening a new file handle.
* add functions for better display of convex meshes
* produce actual triangles for qhull mesh
* Fixed inverted scale for convex meshes inside check
* fix configure config.h.in when paths contain spaces (fix `#9 <https://github.com/ros-planning/geometric_shapes/issues/9>`_)
* Contributors: Acorn, Christian Dornhege, Dave Hershberger, Dirk Thomas, Ioan A Sucan, Michael Ferguson

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
