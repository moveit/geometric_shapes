# Geometric Shapes

This package contains generic definitions of geometric shapes and bodies, as well as tools for operating on shape messages.
Shapes represent only the form of an object.
Bodies are shapes at a particular pose. Routines such as point containment and ray intersections are provided.

Supported shapes:
- sphere
- box
- cone
- cylinder
- mesh

Note: Bodies for meshes compute the convex hull of those meshes in order to provide the point containment / ray intersection routines.

Note: [shape_tools](https://github.com/ros-planning/shape_tools) package was recently merged into this package

## Build Status

Travis CI: [![Build Status](https://travis-ci.org/ros-planning/geometric_shapes.svg?branch=kinetic-devel)](https://travis-ci.org/ros-planning/geometric_shapes)

Devel Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__geometric_shapes__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Jbin_uT64__geometric_shapes__ubuntu_trusty_amd64__binary/)

Debian Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Jdev__geometric_shapes__ubuntu_trusty_amd64)](http://build.ros.org/view/Idev/job/Jdev__geometric_shapes__ubuntu_trusty_amd64/)
