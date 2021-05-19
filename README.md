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

GitHub Actions:
[![Format](https://github.com/ros-planning/geometric_shapes/actions/workflows/format.yaml/badge.svg?branch=melodic-devel)](https://github.com/ros-planning/geometric_shapes/actions/workflows/format.yaml?query=branch%3Amelodic-devel)
[![CI](https://github.com/ros-planning/geometric_shapes/actions/workflows/ci.yaml/badge.svg?branch=melodic-devel)](https://github.com/ros-planning/geometric_shapes/actions/workflows/ci.yaml?query=branch%3Amelodic-devel)

Devel Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__geometric_shapes__ubuntu_bionic__source)](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__geometric_shapes__ubuntu_bionic__source)

Debian Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__geometric_shapes__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__geometric_shapes__ubuntu_bionic_amd64__binary)
