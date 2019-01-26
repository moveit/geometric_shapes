# Migration Notes

API changes in geometric_shapes releases

## ROS Melodic

- There was a change in point containment computations - now all bodies are expected to also contain their surface points. [The old behavior was inconsistent](https://github.com/ros-planning/geometric_shapes/issues/91). Rounding errors may still yield surprising results, though.