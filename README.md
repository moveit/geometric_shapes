geometric_shapes
================

This package contains generic definitions of geometric shapes and bodies. Shapes represent only the form of an object.
Bodies are shapes at a particular pose. Routines such as point containment and ray intersections are provided.

Supported shapes:
- sphere
- box
- cone
- cylinder
- mesh

Note: Bodies for meshes compute the convex hull of those meshes in order to provide the point containment / ray intersection routines.

## Build Status

hydro-devel branch: [![Build Status](https://travis-ci.org/ros-planning/geometric_shapes.png?branch=hydro-devel)](https://travis-ci.org/ros-planning/geometric_shapes)
